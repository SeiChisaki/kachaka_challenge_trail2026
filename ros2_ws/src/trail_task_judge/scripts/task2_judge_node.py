#!/usr/bin/env python3
"""
Task 2 自動採点ノード

/task2/found_garbage に FoundGarbage メッセージが届いたら
内容をログに表示し、画像があればウィンドウに表示します。
"""

import json
from datetime import datetime
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trail_kachaka_msgs.msg import FoundGarbage

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

SAVE_DIR = Path('/app/judge_results/task2')


class Task2Judge(Node):
    def __init__(self):
        super().__init__('task2_judge')
        self.received: list[dict] = []
        SAVE_DIR.mkdir(parents=True, exist_ok=True)

        self.create_subscription(
            FoundGarbage, '/task2/found_garbage', self._callback, 10
        )
        self.status_pub = self.create_publisher(String, '/task2_judge/status', 10)

        self.get_logger().info('=' * 55)
        self.get_logger().info(' Task 2 Judge 起動完了 — ゴミ検出チャレンジ')
        self.get_logger().info(' トピック: /task2/found_garbage (trail_kachaka_msgs/FoundGarbage)')
        self.get_logger().info(f' 画像保存先: {SAVE_DIR}')
        self.get_logger().info('=' * 55)

    def _callback(self, msg: FoundGarbage):
        n = len(self.received) + 1
        item = {
            'id': n,
            'class': msg.garbage_class,
            'confidence': round(float(msg.confidence), 3),
            'x': round(float(msg.robot_x), 2),
            'y': round(float(msg.robot_y), 2),
        }
        self.received.append(item)

        self.get_logger().info(
            f'[受信 #{n}] class={item["class"]}  '
            f'conf={item["confidence"]:.1%}  '
            f'pos=({item["x"]:.1f}, {item["y"]:.1f})'
        )

        self._show_and_save_image(msg, n)
        self._publish_status()

    def _show_and_save_image(self, msg: FoundGarbage, n: int):
        if not CV2_AVAILABLE or len(msg.image.data) == 0:
            return
        try:
            np_arr = np.frombuffer(bytes(msg.image.data), np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                return

            # ウィンドウに表示（手動で閉じるまで残る）
            win_title = f'Task2 Judge — #{n} {msg.garbage_class}'
            cv2.imshow(win_title, img)
            cv2.waitKey(1)

            # ファイルに保存
            timestamp = datetime.now().strftime('%H%M%S')
            filename = SAVE_DIR / f'{n:02d}_{msg.garbage_class}_{timestamp}.jpg'
            cv2.imwrite(str(filename), img)
            self.get_logger().info(f'  画像保存: {filename.name}')
        except Exception as e:
            self.get_logger().warn(f'画像処理エラー: {e}')

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(
            {'task': 2, 'received': len(self.received), 'items': self.received},
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Task2Judge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
