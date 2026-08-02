#!/usr/bin/env python3
"""
Task 1 のウェイポイントを RViz2 に表示するノード

/waypoints (visualization_msgs/MarkerArray) にマーカーを配信します。
RViz2 の設定 (kachaka-nav.rviz) には最初から /waypoints を購読する
MarkerArray Display が入っているので、このノードを起動するだけで表示されます。

【表示されるもの】
- 各ウェイポイントの円柱（Gazebo 上のマーカーと同じ位置・色・サイズ）
- 到達判定の閾値 (2.0m) を示す半透明の円盤
- "WP1: 棚エリア手前" のようなラベル

【使い方】
  # シミュレーションを起動（別ターミナル）
  ros2 launch kachaka_utils launch_sim.launch.py task:=1

  # このノードを実行
  ros2 run trail_kachaka_sample waypoint_marker_publisher.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

# Task 1 のウェイポイント（competition.md / task1_warehouse.sdf と同じ座標・色）
WAYPOINTS = [
    {'x': 5.0,  'y':  3.0, 'label': 'WP1: 棚エリア手前', 'color': (1.0, 0.0, 0.0)},
    {'x': -3.0, 'y':  5.0, 'label': 'WP2: 倉庫中央',     'color': (0.0, 0.8, 0.0)},
    {'x': 8.0,  'y': -4.0, 'label': 'WP3: 奥エリア',     'color': (0.0, 0.0, 1.0)},
    {'x': -6.0, 'y': -2.0, 'label': 'WP4: 左エリア',     'color': (1.0, 1.0, 0.0)},
    {'x': 0.0,  'y':  0.0, 'label': 'WP5: スタート地点', 'color': (1.0, 0.5, 0.0)},
]

# Gazebo 上の円柱マーカーと同じ寸法
CYLINDER_RADIUS = 0.12
CYLINDER_HEIGHT = 0.8

# task1_judge_node.py の REACH_THRESHOLD と同じ値
REACH_THRESHOLD = 2.0


class WaypointMarkerPublisher(Node):
    """ウェイポイントを MarkerArray として配信する。"""

    def __init__(self):
        super().__init__('waypoint_marker_publisher')

        # RViz 側の Display は Volatile なので、後から RViz を開いても
        # 見えるように 1Hz で配信し続ける
        self.pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f'{len(WAYPOINTS)} 個のウェイポイントを /waypoints に配信します'
        )

    def _publish_markers(self):
        msg = MarkerArray()
        for i, wp in enumerate(WAYPOINTS):
            msg.markers.append(self._make_cylinder(i, wp))
            msg.markers.append(self._make_threshold_disk(i, wp))
            msg.markers.append(self._make_label(i, wp))
        self.pub.publish(msg)

    def _make_base_marker(self, namespace: str, marker_id: int) -> Marker:
        """全マーカー共通の設定を持った Marker を作る。"""
        marker = Marker()
        marker.header.frame_id = 'map'
        # stamp を 0 にすると RViz が「最新の TF」で描画してくれる。
        # RViz は use_sim_time:=true で動いているため、実時刻を入れると
        # シミュレーション時刻とずれて表示されなくなる
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _make_cylinder(self, index: int, wp: dict) -> Marker:
        """Gazebo 上のマーカーと同じ円柱。"""
        marker = self._make_base_marker('waypoint', index)
        marker.type = Marker.CYLINDER
        marker.pose.position.x = wp['x']
        marker.pose.position.y = wp['y']
        marker.pose.position.z = CYLINDER_HEIGHT / 2.0
        marker.scale.x = CYLINDER_RADIUS * 2.0
        marker.scale.y = CYLINDER_RADIUS * 2.0
        marker.scale.z = CYLINDER_HEIGHT
        marker.color.r, marker.color.g, marker.color.b = wp['color']
        marker.color.a = 0.8
        return marker

    def _make_threshold_disk(self, index: int, wp: dict) -> Marker:
        """到達判定の範囲 (REACH_THRESHOLD) を示す薄い円盤。"""
        marker = self._make_base_marker('reach_area', index)
        marker.type = Marker.CYLINDER
        marker.pose.position.x = wp['x']
        marker.pose.position.y = wp['y']
        marker.pose.position.z = 0.01
        marker.scale.x = REACH_THRESHOLD * 2.0
        marker.scale.y = REACH_THRESHOLD * 2.0
        marker.scale.z = 0.02
        marker.color.r, marker.color.g, marker.color.b = wp['color']
        marker.color.a = 0.15
        return marker

    def _make_label(self, index: int, wp: dict) -> Marker:
        """円柱の上に出すテキストラベル。"""
        marker = self._make_base_marker('waypoint_label', index)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = wp['label']
        marker.pose.position.x = wp['x']
        marker.pose.position.y = wp['y']
        marker.pose.position.z = CYLINDER_HEIGHT + 0.35
        marker.scale.z = 0.4  # 文字の高さ
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
