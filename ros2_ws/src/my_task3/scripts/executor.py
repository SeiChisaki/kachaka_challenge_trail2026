#!/usr/bin/env python3
"""
Task 3 — 完全探索・分類チャレンジ

【Task 2 との違い】
Task 2 はゴミの座標が公開されていたが、Task 3 は非公開。
倉庫内を自分で網羅的に探索して、bottle / cup / can の 3 種類を
発見・分類しなければならない（計 10 個）。

【このプログラムがやること】
1. Nav2 の起動と自己位置の取得を待つ
2. 倉庫内に等間隔のグリッドを張り、順番に巡回する
3. 各点で 45 度ずつ向き直しては停止し、周囲を見渡しながら YOLO にかける
4. ゴミを見つけたら種類を判定して /task3/found_garbage に報告する
5. 同じゴミを重複報告しないよう、報告済みの位置と比較する

【使い方】
  # シミュレーションを起動（別ターミナル）
  ros2 launch kachaka_utils launch_sim.launch.py task:=3 headless:=True
  pkill -f rviz2          # RViz は CPU を食うので落とす

  # このプログラムを実行
  ros2 run my_task3 executor.py

【採点の確認】
  ros2 topic echo /task3_judge/status
"""

import math
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import tf2_ros
from ultralytics import YOLO

from kachaka_utils.nav_manager import NavManager
from trail_kachaka_msgs.msg import FoundGarbage


# ── 探索範囲 ────────────────────────────────────────────────────────
# ゴミの位置は非公開なので、倉庫の中央部にグリッドを張って網羅的に回る。
# 間隔はカメラでゴミを認識できる距離（実測で 1.5〜2m 程度）を考慮して決めた。
# 3.0m 間隔なら、グリッドのどのマスの中にゴミがあっても
# 最寄りの探索点から最大 2.1m 以内に収まる（対角線の半分 = 1.5*sqrt(2)）。
# 各点で 8 方向の見渡しに 30 秒以上かかるので、点数を増やしすぎると
# 全体の所要時間が現実的でなくなる。取りこぼすようなら 2.0m 間隔に狭める。
GRID_X = [-6.0, -3.0, 0.0, 3.0, 6.0]
GRID_Y = [-4.5, -1.5, 1.5, 4.5]

# ── YOLO の設定 ────────────────────────────────────────────────────
MODEL_PATH = '/app/yolo11n.pt'
CONF_THRESHOLD = 0.05       # Gazebo の 3D モデルは信頼度が非常に低く出る
IMG_SIZE = 1280           # 既定の 640 だと縮小されてゴミを検出できない

# COCO のクラス ID → 報告するゴミの種類。
#
# competition.md は bottle=39 / cup=41 / can=46 としているが、
# COCO に can というクラスは存在せず 46 は banana である。
# Task 2 で Coke Can を実測したところ 46 では一度も検出されず、
# fire hydrant(10) / bottle(39) / cup(41) / vase(75) として出ていた。
# ここでは COCO の意味に素直に従い、缶らしい形として出る
# fire hydrant と vase を can に寄せている
CLASS_MAP = {
    39: 'bottle',       # bottle       → Beer モデル
    41: 'cup',          # cup          → Plastic Cup モデル
    10: 'can',          # fire hydrant → Coke Can が最も安定して出るクラス
    46: 'can',          # banana       → 資料が can としているクラス
    75: 'can',          # vase         → Coke Can が出ることがある
}

# ── 探索動作の設定 ──────────────────────────────────────────────────
SPIN_SPEED = 0.5            # その場回転の角速度 [rad/s]
SPIN_MARGIN = 1.15          # 一周ぶんに対する余裕（取りこぼし防止）
INFER_INTERVAL = 0.6        # 回転中に推論する間隔 [s]
ARRIVAL_TOLERANCE = 3.0     # 探索点にこれ以上近づけなければスキップ [m]

# ── 見渡し動作の設定 ────────────────────────────────────────────────
# 探索点に着いたら、一定角度ずつ向き直しては停止して撮影する。
# 回しっぱなしだと画像がブレて YOLO の信頼度が落ちるため、
# 「止まってから撮る」ほうが検出率が上がる。
LOOK_DIRECTIONS = 8         # 何方向を見るか（8 なら 45 度刻みで一周）
LOOK_SETTLE_SEC = 0.5       # 停止してから撮影するまでの静止待ち [s]
LOOK_SHOTS = 2              # 1 方向あたりの推論回数
YAW_TOLERANCE = 0.09        # 目標の向きに対する許容誤差 [rad]（約 5 度）
TURN_SPEED_MAX = 0.6        # 向き直すときの最大角速度 [rad/s]
TURN_SPEED_MIN = 0.15       # 静止摩擦に負けないための最小角速度 [rad/s]
TURN_TIMEOUT = 8.0          # 1 方向あたりの向き直しタイムアウト [s]

# 同じ種類のゴミをこの距離以内で再検出しても、同一物とみなして報告しない [m]
DEDUP_RADIUS = 2.5

# 検出できた画像の保存先（None にすると保存しない）
SAVE_DIR = Path('/app/judge_results/task3_found')

# デバッグ用：推論にかけた画像の保存先。
# 不要になったら DEBUG_DIR = None にすれば保存を止められる。
DEBUG_DIR = Path('/app/judge_results/task3_debug')

# 上記のうち、この class id が検出されたフレームだけを保存する。
# Coke Can は COCO に can クラスが無いため代替クラスとして出るので、
# 「実際にどのクラスで出ているのか」を目視で確かめるための絞り込み。
# 全フレームを保存したい場合は None にする。
DEBUG_SAVE_CLASS_IDS = {10, 46, 75}     # fire hydrant / banana / vase

CMD_VEL_TOPIC = '/kachaka/manual_control/cmd_vel'


def normalize_angle(angle: float) -> float:
    """角度を -pi 〜 +pi の範囲に丸める。"""
    return math.atan2(math.sin(angle), math.cos(angle))


class Task3Executor(Node):
    """倉庫をグリッド探索しながら 3 種類のゴミを発見・分類するノード。"""

    def __init__(self):
        super().__init__('task3_executor')

        self.nav = NavManager(self)

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

        self.found_pub = self.create_publisher(FoundGarbage, '/task3/found_garbage', 10)

        # その場回転は Nav2 を経由せず速度指令を直接出す。
        # Nav2 の go_to() で 1 方向ずつ向き直すと 1 点あたり 1 分近くかかるが、
        # 回しながら連続で推論すれば十数秒で全周を見られる
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.get_logger().info(f'YOLO モデルを読み込みます: {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)

      
        # 報告済みのゴミ: [(種類, x, y, yaw(向き)), ...]
        self.reported: list[tuple[str, float, float, float]] = []

        # デバッグ保存した枚数（ファイル名の連番に使う）
        self.debug_count = 0
   
    # ── コールバック ──────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage):
        self.latest_image_msg = msg

    # ── メイン処理 ────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Nav2 の起動を待機中... (1〜2 分かかることがあります)')
        self.nav.wait_until_nav2_active()

        self.get_logger().info('自己位置 (map→base_footprint) の取得を待機中...')
        if not self._wait_for_localization():
            self.get_logger().error('自己位置を取得できませんでした。AMCL が map→odom を配信していません')
            self.get_logger().error('シミュレーションを再起動し、RViz を終了して CPU 負荷を下げてください')
            return

        waypoints = self._build_route()
        self.get_logger().info('=' * 55)
        self.get_logger().info(f' Task 3 探索開始 — 探索点 {len(waypoints)} 箇所')
        self.get_logger().info('=' * 55)

        for i, (wx, wy) in enumerate(waypoints):
            label = f'探索点 {i + 1}/{len(waypoints)} ({wx:.1f}, {wy:.1f})'
            self.get_logger().info(f'→ {label} へ移動します')

            # 現在地から目的地への角度（yaw）を計算する
            target_yaw = 0.0
            robot_pose = self._get_robot_pose()
            if robot_pose is not None:
                rx, ry, _ = robot_pose
                target_yaw = math.atan2(wy - ry, wx - rx)

            # 角度(target_yaw)も指定して移動する
            self.nav.go_to(wx, wy, yaw=target_yaw)

            distance = self._distance_to(wx, wy)

            if distance is None or distance > ARRIVAL_TOLERANCE:
                shown = f'{distance:.1f}m' if distance is not None else '不明'
                self.get_logger().warn(f'  到達できませんでした（残り {shown}）— 次へ')
                continue

            found = self._look_around()
            self.get_logger().info(
                f'  {label} で {found} 個の新規ゴミを報告（累計 {len(self.reported)}/10）'
            )

        self._print_summary()

    def _build_route(self) -> list[tuple[float, float]]:
        """グリッドを蛇行（ブーストロフェドン）順に並べた巡回路を作る。

        単純に二重ループで並べると行の端から端へ戻る無駄な移動が生じる。
        1 行ごとに向きを反転させると移動距離が短くなる。
        """
        route = []
        for row, y in enumerate(GRID_Y):
            xs = GRID_X if row % 2 == 0 else list(reversed(GRID_X))
            for x in xs:
                route.append((x, y))
        return route

    def _look_around(self) -> int:
        """その場で少しずつ向き直しながら、停止した状態で周囲を見渡す。

        LOOK_DIRECTIONS 方向に等分して一周する。各方向では
        いったん止まって車体の揺れが収まるのを待ってから推論するので、
        回しながら撮るよりブレの少ない画像で判定できる。

        自己位置が取れない場合は、TF に頼らない従来の連続回転に切り替える。

        Returns:
            この探索点で新規に報告したゴミの数
        """
        start_pose = self._get_robot_pose()
        if start_pose is None:
            self.get_logger().warn('  自己位置を取得できないため、連続回転で見渡します')
            return self._spin_and_scan()

        start_yaw = start_pose[2]
        step = 2 * math.pi / LOOK_DIRECTIONS
        new_count = 0

        try:
            for i in range(LOOK_DIRECTIONS):
                target_yaw = normalize_angle(start_yaw + i * step)

                # 0 番目は今向いている方向そのものなので回さずに撮る
                if i > 0 and not self._turn_to_yaw(target_yaw):
                    self.get_logger().warn(
                        f'  方向 {i + 1}/{LOOK_DIRECTIONS} '
                        f'({math.degrees(target_yaw):.0f}度) へ向き直せませんでした'
                    )

                # 停止して揺れが収まるのを待つ（この間も画像は受信し続ける）
                self._spin_for(LOOK_SETTLE_SEC)

                for _ in range(LOOK_SHOTS):
                    for garbage_class, confidence in self._detect():
                        if self._report_if_new(garbage_class, confidence):
                            new_count += 1
                    self._spin_for(INFER_INTERVAL)
        finally:
            self.cmd_vel_pub.publish(Twist())

        return new_count

    def _turn_to_yaw(self, target_yaw: float) -> bool:
        """TF の yaw を見ながら、指定した向きになるまでその場回転する。

        角度誤差に比例した速度で回し、誤差が YAW_TOLERANCE 未満になったら止める。
        速度が小さすぎると実機・シミュレータともに動き出さないので
        TURN_SPEED_MIN で下限を設けている。

        Returns:
            True: 目標の向きに到達 / False: タイムアウトまたは自己位置ロスト
        """
        end = self.get_clock().now().nanoseconds + int(TURN_TIMEOUT * 1e9)
        twist = Twist()

        try:
            while self.get_clock().now().nanoseconds < end:
                pose = self._get_robot_pose()
                if pose is None:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    continue

                error = normalize_angle(target_yaw - pose[2])
                if abs(error) < YAW_TOLERANCE:
                    return True

                speed = min(max(abs(error) * 1.5, TURN_SPEED_MIN), TURN_SPEED_MAX)
                twist.angular.z = math.copysign(speed, error)
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)

            return False
        finally:
            # 目標に着いても失敗しても、必ず停止指令を出す
            self.cmd_vel_pub.publish(Twist())

    def _spin_and_scan(self) -> int:
        """その場で 1 回転しながら連続で推論し、新しいゴミを報告する。

        Returns:
            この探索点で新規に報告したゴミの数
        """
        duration = (2 * math.pi / SPIN_SPEED) * SPIN_MARGIN
        end = self.get_clock().now().nanoseconds + int(duration * 1e9)
        next_infer = 0
        new_count = 0

        twist = Twist()
        twist.angular.z = SPIN_SPEED

        try:
            while self.get_clock().now().nanoseconds < end:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)

                now = self.get_clock().now().nanoseconds
                if now < next_infer:
                    continue
                next_infer = now + int(INFER_INTERVAL * 1e9)

                for garbage_class, confidence in self._detect():
                    if self._report_if_new(garbage_class, confidence):
                        new_count += 1
        finally:
            # 回転を必ず止める（例外で抜けても止まるように finally に置く）
            self.cmd_vel_pub.publish(Twist())
            self._spin_for(0.5)
            self.cmd_vel_pub.publish(Twist())

        return new_count

   
    def _detect(self) -> list[tuple[str, float]]:
        """最新のカメラ画像からゴミの候補を取り出す。"""
        image_msg = self.latest_image_msg
        if image_msg is None:
            # 診断1：そもそも画像が1枚も届いていない
            self.get_logger().warn('【診断】カメラ画像を受信できていません！ (TopicかQoSの問題)')
            return []

        image = cv2.imdecode(
            np.frombuffer(bytes(image_msg.data), np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            self.get_logger().warn('【診断】画像の変換に失敗しました。')
            return []

        # IMG_SIZEが大きすぎると検出できないことがあるため、標準の640に固定してテスト
        results = self.model(image, conf=CONF_THRESHOLD, imgsz=640, verbose=False)

        if len(results[0].boxes) == 0:
            # 診断2：画像は届いているが、YOLOが1個も物体を見つけられない
            self.get_logger().info('【診断】画像は受信済ですが、YOLOが何も検出していません（箱0個）')

        self._save_debug_image(image, results[0])

        best: dict[str, float] = {}
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            conf = float(box.conf[0])

            # 診断3：YOLOは何かを見つけているが、クラスマップに無いゴミ（ID）になっている
            self.get_logger().info(f'【診断】YOLOが物体を発見 -> ID:{class_id}, 信頼度:{conf:.2f}')

            garbage_class = CLASS_MAP.get(class_id)
            if garbage_class is None:
                continue
            if conf > best.get(garbage_class, 0.0):
                best[garbage_class] = conf

        # ここで `_last_image_msg_used` は使っていないので削除（エラー防止）
        return sorted(((c, v) for c, v in best.items()), key=lambda t: -t[1])
 
    def _report_if_new(self, garbage_class: str, confidence: float) -> bool:
        robot_pose = self._get_robot_pose() # (x, y, yaw) を取得
        if robot_pose is None:
            return False
        rx, ry, ryaw = robot_pose

        for prev_class, px, py, pyaw in self.reported:
            if prev_class != garbage_class:
                continue
            
            # 過去に報告した時のロボットとの「距離」
            dist = math.hypot(rx - px, ry - py)
            
            # 過去に報告した時のロボットとの「向きの差(ラジアン)」
            yaw_diff = abs(normalize_angle(ryaw - pyaw))

            # 「2.5m以内の同じ場所」かつ「向いている方向の差が45度(約0.8rad)以内」なら重複とみなす
            if dist < DEDUP_RADIUS and yaw_diff < 0.8:
                return False

        msg = FoundGarbage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.garbage_class = garbage_class
        msg.confidence = float(confidence)
        msg.robot_x = float(rx)
        msg.robot_y = float(ry)
        if self.latest_image_msg is not None:
            msg.image = self.latest_image_msg

        self.found_pub.publish(msg)
        self.reported.append((garbage_class, rx, ry, ryaw)) # yawも保存する
        self.get_logger().info(
            f'  ★ {garbage_class} を発見 (信頼度 {confidence:.1%}) '
            f'位置 ({rx:.1f}, {ry:.1f}) — 累計 {len(self.reported)} 個'
        )
        self._save_image(garbage_class, confidence)
        return True
 

   
    def _print_summary(self):
        counts: dict[str, int] = {}
        for garbage_class, _, _, _ in self.reported: # アンダースコアを3つにする
            counts[garbage_class] = counts.get(garbage_class, 0) + 1

        self.get_logger().info('=' * 55)
        self.get_logger().info(f' 探索完了 — 合計 {len(self.reported)} 個を報告')
        self.get_logger().info(
            f'   bottle={counts.get("bottle", 0)} '
            f'cup={counts.get("cup", 0)} '
            f'can={counts.get("can", 0)}'
        )
        self.get_logger().info(' 正解は bottle=4 / cup=3 / can=3 の計 10 個')
        self.get_logger().info('=' * 55)
   

    # ── 補助メソッド ──────────────────────────────────────────────

    def _save_debug_image(self, image, result):
        """推論にかけた画像をそのまま保存する（デバッグ用）。

        DEBUG_SAVE_CLASS_IDS が指定されている場合、そのクラスが
        検出されたフレームだけを残す。ファイル名に該当したクラス ID を
        入れておくと、「この見え方のときに YOLO が何を返したか」を
        後から突き合わせられる。
        """
        if DEBUG_DIR is None:
            return

        ids = sorted({int(box.cls[0]) for box in result.boxes})
        if DEBUG_SAVE_CLASS_IDS is not None:
            ids = [i for i in ids if i in DEBUG_SAVE_CLASS_IDS]
            if not ids:
                return

        DEBUG_DIR.mkdir(parents=True, exist_ok=True)
        tag = '-'.join(str(i) for i in ids) if ids else 'none'

        robot_pose = self._get_robot_pose()
        where = f'{robot_pose[0]:.1f}_{robot_pose[1]:.1f}' if robot_pose else 'unknown'

        path = DEBUG_DIR / f'{self.debug_count:04d}_{where}_ids-{tag}.jpg'
        cv2.imwrite(str(path), image)
        self.debug_count += 1

    def _save_image(self, garbage_class: str, confidence: float):
        """報告したゴミの画像を残す（後から誤検出を確認するため）。"""
        if SAVE_DIR is None or self.latest_image_msg is None:
            return
        image = cv2.imdecode(
            np.frombuffer(bytes(self.latest_image_msg.data), np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            return
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        n = len(self.reported)
        path = SAVE_DIR / f'{n:02d}_{garbage_class}_{confidence:.2f}.jpg'
        cv2.imwrite(str(path), image)

    def _wait_for_localization(self, timeout_sec: float = 30.0) -> bool:
        """map→base_footprint が引けるようになるまで待つ。"""
        end = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self.get_clock().now().nanoseconds < end:
            if self._get_robot_pose() is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _distance_to(self, gx: float, gy: float) -> float | None:
        robot = self._get_robot_pose()
        if robot is None:
            return None
        rx, ry, _ = robot
        return math.hypot(gx - rx, gy - ry)

  
    def _get_robot_pose(self) -> tuple[float, float, float] | None:
        """map 座標系でのロボットの現在位置と向き(yaw)を tf2 から取得する。"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # クォータニオンから yaw (Z軸の回転角) を計算
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
            return x, y, yaw
        except tf2_ros.TransformException:
            return None
 
    def _spin_for(self, seconds: float):
        """指定秒数のあいだコールバックを処理する。"""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Task3Executor()
    try:
        node.run()
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())   # 中断時も回転を止める
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
