#!/usr/bin/env python3
"""
Task 1 自動採点ノード

ロボットの位置を tf2 (map → base_footprint) から取得し、
各ウェイポイントへの到達を自動で検出・記録します。

【採点ロジック】
- ロボットがウェイポイントから REACH_THRESHOLD [m] 以内に入ったら到達とみなす
- 全WP到達で完了ログを出力

【出力トピック】
/task1_judge/status (std_msgs/String): JSON形式でスコアを配信
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros


WAYPOINTS = [
    {"id": 1, "x": 5.0,  "y":  3.0, "label": "WP1: 棚エリア手前"},
    {"id": 2, "x": -3.0, "y":  5.0, "label": "WP2: 倉庫中央"},
    {"id": 3, "x": 8.0,  "y": -4.0, "label": "WP3: 奥エリア"},
    {"id": 4, "x": -6.0, "y": -2.0, "label": "WP4: 左エリア"},
    {"id": 5, "x": 0.0,  "y":  0.0, "label": "WP5: スタート地点"},
]

REACH_THRESHOLD = 2.0  # ウェイポイント円柱の半径を考慮して 2.0m


class Task1Judge(Node):
    def __init__(self):
        super().__init__('task1_judge')
        self.reached = [False] * len(WAYPOINTS)
        self.robot_x = None
        self.robot_y = None

        # tf2_ros.Buffer で map→base_footprint を正しく取得する
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, '/task1_judge/status', 10)

        # 0.5秒ごとにロボット位置を取得して判定
        self.create_timer(0.5, self._poll_pose)
        self.create_timer(5.0, self._report_timer)

        self.get_logger().info('=' * 55)
        self.get_logger().info(' Task 1 Judge 起動完了 — ウェイポイントナビゲーション')
        self.get_logger().info(f' 目標: {len(WAYPOINTS)} ウェイポイントを巡回')
        self.get_logger().info(f' 到達判定閾値: {REACH_THRESHOLD}m')
        self.get_logger().info('=' * 55)

    def _poll_pose(self):
        """map→base_footprint の変換からロボットの実際の位置を取得する。"""
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
            )
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            self._check_waypoints()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def _check_waypoints(self):
        if self.robot_x is None:
            return
        for i, wp in enumerate(WAYPOINTS):
            if self.reached[i]:
                continue
            dist = math.sqrt((self.robot_x - wp['x']) ** 2 + (self.robot_y - wp['y']) ** 2)
            if dist < REACH_THRESHOLD:
                self.reached[i] = True
                count = sum(self.reached)
                self.get_logger().info(
                    f'[✓] {wp["label"]} 到達！'
                    f' (距離: {dist:.2f}m, {count}/{len(WAYPOINTS)} WP完了)'
                )
                self._publish_status()
                if count == len(WAYPOINTS):
                    self.get_logger().info('★ 全ウェイポイント到達完了！ お疲れ様でした ★')

    def _report_timer(self):
        count = sum(self.reached)
        if self.robot_x is not None:
            pos_str = f'現在位置: ({self.robot_x:.2f}, {self.robot_y:.2f})'
        else:
            pos_str = '現在位置: TF 取得中...'

        if count < len(WAYPOINTS):
            remaining = [wp['label'] for wp, r in zip(WAYPOINTS, self.reached) if not r]
            self.get_logger().info(
                f'進捗: {count}/{len(WAYPOINTS)} | {pos_str} | 残り: {remaining}'
            )
        else:
            self.get_logger().info(f'★ 全ウェイポイント到達済み ★ | {pos_str}')

    def _publish_status(self):
        reached_labels = [wp['label'] for wp, r in zip(WAYPOINTS, self.reached) if r]
        score = sum(self.reached) * 20
        status = {
            'task': 1,
            'reached': sum(self.reached),
            'total': len(WAYPOINTS),
            'score': score,
            'reached_waypoints': reached_labels,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Task1Judge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
