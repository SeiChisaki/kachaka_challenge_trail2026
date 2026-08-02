#!/usr/bin/env python3
"""
Task 2 — ゴミ検出チャレンジ

【このプログラムがやること】
1. Nav2 の起動を待つ
2. 公開されている 10 箇所のゴミ座標を順番に巡回する
3. 各地点で少し手前に停まり、その場で向きを変えながらカメラ画像を YOLO にかける
4. 缶（YOLO クラス ID 46）を見つけたら /task2/found_garbage に報告する

【使い方】
  # シミュレーションを起動（別ターミナル）
  ros2 launch kachaka_utils launch_sim.launch.py task:=2

  # このプログラムを実行
  ros2 run my_task2 executor.py

【採点の確認】
  ros2 topic echo /task2_judge/status
"""

import math
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
import tf2_ros
from ultralytics import YOLO

from kachaka_utils.nav_manager import NavManager
from trail_kachaka_msgs.msg import FoundGarbage


# ゴミの座標（competition.md で公開されている 10 箇所）
GARBAGE_POINTS = [
    (5.5, 2.5),
    (4.5, 4.0),
    (-3.0, 4.5),
    (-2.0, 6.0),
    (7.5, -3.5),
    (9.0, -5.0),
    (-6.5, -0.5),
    (-7.0, -3.0),
    (2.5, 3.5),
    (-0.5, 2.0),
]

# YOLO の設定
MODEL_PATH = '/app/yolo11n.pt'

# 缶とみなす COCO クラス ID。
#
# competition.md と task2_warehouse.sdf には「クラス ID 46 = can」とあるが、
# COCO に can というクラスは存在せず 46 は banana。実際にシミュレータの
# カメラ画像で測定したところ、Coke Can は 46 では一度も検出されなかった。
#
# 缶の位置に出るクラスを実測した結果（缶の枠と完全に一致するもの）:
#   fire hydrant(10) 0.20〜0.43  ← 最も安定して高い
#   bottle(39)       0.11
#   cup(41)          0.05〜0.37
#   vase(75)         0.08
# 倉庫ワールドには消火栓も花瓶も存在しないので、これらを拾っても誤報にならない。
# 逆に person / chair は実物がワールドに置かれているので入れてはいけない
CAN_CLASS_IDS = (10, 39, 41, 46, 75)

# Gazebo の 3D モデルは信頼度が非常に低く出る（実測で 0.05〜0.11 程度）
CONF_THRESHOLD = 0.05

# 推論時の入力解像度。既定の 640 だと 1280x720 の画像が縮小されて
# 缶がほとんど検出できない。ネイティブ解像度で推論すると大幅に改善する
IMG_SIZE = 1280

# 探索の設定
STANDOFF = 1.5              # ゴミの何 m 手前に停まるか
                            # 近すぎる（1m 未満）と缶が画面いっぱいになり逆に検出されない
SCAN_YAWS = [0.0, 0.6, -0.6]  # 停止後、正面からこの角度だけ首を振って探す [rad]
SETTLE_SEC = 2.0            # 停止後、カメラ画像が更新されるのを待つ時間 [s]
ARRIVAL_TOLERANCE = 3.0     # 目標からこれ以上離れていたら「未到達」として検出をスキップする [m]

# 検出できなかったときの画像の保存先（None にすると保存しない）
MISS_DIR = Path('/app/judge_results/task2_miss')


class Task2Executor(Node):
    """ゴミ座標を巡回しながら缶を検出して報告するノード。"""

    def __init__(self):
        super().__init__('task2_executor')

        self.nav = NavManager(self)

        # ロボットの実際の位置は tf2 の map→base_footprint から取る。
        # NavManager.get_current_pose_stamped() は map→odom（自己位置の補正量）を
        # 返す実装になっていて、ロボットの位置そのものではない点に注意
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # カメラ画像は QoS を BEST_EFFORT にしないと受信できない
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(
            CompressedImage,
            '/kachaka/front_camera/image_raw/compressed',
            self._image_callback,
            qos,
        )
        self.latest_image_msg: CompressedImage | None = None

        self.found_pub = self.create_publisher(FoundGarbage, '/task2/found_garbage', 10)

        self.get_logger().info(f'YOLO モデルを読み込みます: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)

        self.found_count = 0
        self._point_index = 0   # いま処理中のゴミ地点の番号（1 始まり）
        self._miss_index = 0    # その地点で何枚目の失敗画像か

    # ── コールバック ──────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage):
        """カメラ画像を受け取って保持しておく（推論は巡回ループ側で行う）。"""
        self.latest_image_msg = msg

    # ── メイン処理 ────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Nav2 の起動を待機中... (1〜2 分かかることがあります)')
        self.nav.wait_until_nav2_active()
        self.get_logger().info('Nav2 準備完了！ゴミ探索を開始します')

        for i, (gx, gy) in enumerate(GARBAGE_POINTS):
            label = f'ゴミ地点 {i + 1}/{len(GARBAGE_POINTS)} ({gx:.1f}, {gy:.1f})'
            self.get_logger().info(f'→ {label} へ向かいます')

            # 失敗画像のファイル名に使う。地点ごとに枚数を数え直す
            self._point_index = i + 1
            self._miss_index = 0
        
            self._approach(gx, gy)
     

            # Nav2 が経路を諦めると、目的地から遠く離れた場所で検出処理に
            # 入ってしまい「別の地点の缶」をこの地点の成果として数えてしまう。
            # 実際に近くまで来られたときだけ検出を行う
            distance = self._distance_to(gx, gy)
            if distance is None or distance > ARRIVAL_TOLERANCE:
                shown = f'{distance:.1f}m' if distance is not None else '不明'
                self.get_logger().warn(f'✗ {label} に到達できませんでした（残り {shown}）')
                continue

            if self._scan_and_report(gx, gy):
                self.get_logger().info(f'✓ {label} で缶を発見しました')
            else:
                self.get_logger().warn(f'✗ {label} では缶を検出できませんでした')

        self.get_logger().info('=' * 55)
        self.get_logger().info(
            f' 探索完了 — {self.found_count}/{len(GARBAGE_POINTS)} 個の缶を報告しました'
        )
        self.get_logger().info('=' * 55)

    def _approach(self, gx: float, gy: float):
        """ゴミの STANDOFF [m] 手前まで移動し、ゴミの方を向く。"""
        robot = self._get_robot_xy()
        self.get_logger().info(f"robot:{robot}")
        if robot is None:
            self.get_logger().info(f"ここでrobotが取得できていない")
            # 位置が取れないときは仕方ないのでゴミ座標をそのまま目標にする
            self.nav.go_to(gx, gy)
            return

        rx,ry = robot
        dx, dy = gx - rx, gy - ry
        self.get_logger().info(f"移動距離は{dx},{dy}")
        distance = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)

        if distance <= STANDOFF:
            # すでに十分近い場合は、その場でゴミの方を向くだけ
            self.nav.go_to(rx, ry, yaw)
            return

        # ロボット → ゴミ を結ぶ直線上で、ゴミの STANDOFF 手前の点を目標にする
        approach_x = gx - STANDOFF * dx / distance
        approach_y = gy - STANDOFF * dy / distance

        if not self.nav.go_to(approach_x, approach_y, yaw):
            self.get_logger().warn('手前の地点に到達できなかったので、直接ゴミ座標を目指します')
            self.nav.go_to(gx, gy, yaw)

    def _scan_and_report(self, gx: float, gy: float) -> bool:
        """その場で首を振りながら缶を探し、見つけたら 1 回だけ報告する。"""
        robot = self._get_robot_xy()
        base_yaw = math.atan2(gy - robot[1], gx - robot[0]) if robot else 0.0

        for offset in SCAN_YAWS:
            if robot is not None:
                # offset=0.0 のときも必ず向き直す。
                # _approach() で指定した yaw は「移動前の位置から見たゴミの方向」で、
                # Nav2 は目標の許容誤差内で止まるため、到着地点から見ると
                # ゴミの方向がずれている。実際、向き直さないまま撮ると
                # 缶が画面の端で見切れたり画角の外に出てしまう
                self.nav.go_to(robot[0], robot[1], base_yaw + offset)

            # 回転直後は古い画像が残っているので、新しいフレームが届くまで待つ
            self._spin_for(SETTLE_SEC)

            detection = self._detect_can()
            if detection is None:
                continue

            confidence, image_msg = detection
            self._report(confidence, image_msg)
            return True

        return False

    def _detect_can(self) -> tuple[float, CompressedImage] | None:
        """最新のカメラ画像に缶が写っていれば (信頼度, 画像) を返す。"""
        image_msg = self.latest_image_msg
        if image_msg is None:
            self.get_logger().warn('カメラ画像がまだ届いていません')
            return None

        # CompressedImage → numpy 配列 → cv2 画像
        np_arr = np.frombuffer(bytes(image_msg.data), np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return None

        results = self.model(image, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)

        best_conf = 0.0
        best_name = ''
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            if class_id not in CAN_CLASS_IDS:
                continue
            conf = float(box.conf[0])
            if conf > best_conf:
                best_conf, best_name = conf, self.model.names[class_id]

        if best_conf == 0.0:
            # 何も拾えなかったときは、何が写っていたのかをログに残しておくと
            # 閾値やクラス ID を調整するときの手がかりになる
            others = sorted(
                ((float(b.conf[0]), self.model.names[int(b.cls[0])])
                 for b in results[0].boxes),
                reverse=True,
            )[:3]
            if others:
                detail = ', '.join(f'{n} {c:.2f}' for c, n in others)
                self.get_logger().info(f'  缶なし（画面内の候補: {detail}）')
            self._save_miss_image(image)
            return None

        self.get_logger().info(f'  缶を検出 ({best_name} として 信頼度 {best_conf:.1%})')
        return best_conf, image_msg

    def _report(self, confidence: float, image_msg: CompressedImage):
        """採点ノードに発見を報告する。"""
        robot = self._get_robot_xy()

        msg = FoundGarbage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.garbage_class = 'can'
        msg.confidence = float(confidence)
        msg.robot_x = float(robot[0]) if robot else 0.0
        msg.robot_y = float(robot[1]) if robot else 0.0
        msg.image = image_msg

        self.found_pub.publish(msg)
        self.found_count += 1
        self.get_logger().info(
            f'  → 報告しました (合計 {self.found_count} 個, '
            f'位置 ({msg.robot_x:.1f}, {msg.robot_y:.1f}))'
        )

    # ── 補助メソッド ──────────────────────────────────────────────

    def _save_miss_image(self, image) -> None:
        """検出できなかったときの画像を残す。

        「缶が視界に入っていないのか」「写っているが誤分類なのか」を
        後から確認するために使う。MISS_DIR を None にすると保存しない。

        ファイル名は miss_p03_02.jpg のように
        「ゴミ地点 3 の 2 枚目」が分かる形式にしてある。
        """
        if MISS_DIR is None:
            return
        MISS_DIR.mkdir(parents=True, exist_ok=True)
        self._miss_index += 1
        path = MISS_DIR / f'miss_p{self._point_index:02d}_{self._miss_index:02d}.jpg'
        cv2.imwrite(str(path), image)
        self.get_logger().info(f'  未検出の画像を保存: {path.name}')

    def _distance_to(self, gx: float, gy: float) -> float | None:
        """現在位置から目標地点までの距離。位置が取れなければ None。"""
        robot = self._get_robot_xy()
        if robot is None:
            return None
        return math.hypot(gx - robot[0], gy - robot[1])

    def _get_robot_xy(self) -> tuple[float, float] | None:
        """map 座標系でのロボットの現在位置を tf2 から取得する。"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            self.get_logger().info(f"現在のrobotの位置{t.transform.translation.x},{t.transform.translation.y}")
            
            return t.transform.translation.x, t.transform.translation.y
        except tf2_ros.TransformException:
            return None

    def _spin_for(self, seconds: float):
        """指定秒数のあいだコールバック（カメラ画像の受信）を処理する。"""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Task2Executor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
