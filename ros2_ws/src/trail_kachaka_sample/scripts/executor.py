#!/usr/bin/env python3
"""
サンプルコード — Task 1 ウェイポイントナビゲーション

このコードは Task 1 の実装例です。
自分のパッケージを作るときの参考にしてください。

【このコードがやること】
1. カメラトピックの購読（動作確認用）
2. Nav2 が起動するまで待機
3. 5 つのウェイポイントを順番に巡回

【使い方】
  # シミュレーションを起動（別ターミナル）
  ros2 launch kachaka_utils launch_sim.launch.py task:=1

  # このサンプルを実行
  ros2 run trail_kachaka_sample executor.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from kachaka_utils.nav_manager import NavManager

# Task 1 のウェイポイント（competition.md と同じ座標）
WAYPOINTS = [
    (5.0,  3.0),   # WP1: 棚エリア手前
    (-3.0, 5.0),   # WP2: 倉庫中央
    (8.0, -4.0),   # WP3: 奥エリア
    (-6.0, -2.0),  # WP4: 左エリア
    (0.0,  0.0),   # WP5: スタート地点に帰還
]


class SampleNavigator(Node):
    """Task 1 のシンプルな実装例。"""

    def __init__(self):
        super().__init__('sample_navigator')

        # NavManager を使ってナビゲーションを制御する
        self.nav = NavManager(self)

        # カメラ画像を購読してみる（動作確認）
        # QoS は BEST_EFFORT を使うことで画像トピックを安定して受け取れる
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(
            CompressedImage,
            '/kachaka/front_camera/image_raw/compressed',
            self._camera_callback,
            qos,
        )

        self.get_logger().info('サンプルナビゲーターが起動しました')

    def _camera_callback(self, msg: CompressedImage):
        """カメラ画像を受信したときに呼ばれる。"""
        # throttle_duration_sec=5.0 で 5 秒に 1 回だけログを出す
        self.get_logger().info(
            f'カメラ画像を受信中 ({len(msg.data)} bytes)',
            throttle_duration_sec=5.0,
        )

    def run(self):
        """ウェイポイントを順番に巡回する。"""
        # Nav2 が完全に起動するまで待つ（必須: これを忘れると go_to() が失敗する）
        self.get_logger().info('Nav2 の起動を待機中... (1〜2 分かかることがあります)')
        self.nav.wait_until_nav2_active()
        self.get_logger().info('Nav2 準備完了！ウェイポイント巡回を開始します')

        for i, (x, y) in enumerate(WAYPOINTS):
            self.get_logger().info(f'→ WP{i + 1} ({x:.1f}, {y:.1f}) に向かいます')

            # go_to() は到達または失敗するまでブロックする（同期的に動く）
            success = self.nav.go_to(x, y)

            if success:
                self.get_logger().info(f'✓ WP{i + 1} 到達成功！')
            else:
                self.get_logger().warn(f'✗ WP{i + 1} 到達失敗。次のウェイポイントへ進みます。')

        self.get_logger().info('全ウェイポイントの巡回が完了しました！')


def main():
    rclpy.init()
    node = SampleNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
