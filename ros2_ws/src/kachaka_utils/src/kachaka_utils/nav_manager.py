import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavManager:
    """Nav2 を使ったロボットナビゲーションのラッパークラス。

    使い方:
        nav = NavManager(self)
        nav.wait_until_nav2_active()   # Nav2の起動待ち
        nav.go_to(5.0, 3.0)            # 指定座標へ移動
        nav.cancel_nav()               # 移動をキャンセル
        nav.get_current_pose_stamped() # 現在位置を取得
    """

    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.initial_pose = Pose()
        self.initial_pose_received = False
        self.goal_handle = None
        self.result_future = None
        self.status: GoalStatus | None = None
        self.current_pose: Pose | None = None

        self._nav_client = ActionClient(self.parent_node, NavigateToPose, "navigate_to_pose")
        self.parent_node.create_subscription(TFMessage, "/tf", self._tf_callback, 10)
        self._initial_pose_pub = self.parent_node.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )

    # ── 公開API ──────────────────────────────────────────────────

    def wait_until_nav2_active(self):
        """Nav2 が使用可能になるまで待機する。プログラム開始時に必ず呼ぶ。"""
        self._wait_for_node_to_activate("amcl")
        self._wait_for_initial_pose()
        self._wait_for_node_to_activate("bt_navigator")
        self.info("Nav2 is ready!")

    def set_initial_pose(self, initial_pose: Pose):
        """AMCL に初期位置を通知する。デフォルトは原点(0, 0)。"""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._publish_initial_pose()

    def go_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """指定座標にナビゲートし、完了まで待機する。

        Args:
            x: 目標のX座標 [m]
            y: 目標のY座標 [m]
            yaw: 目標の向き [rad]（省略時は 0.0）

        Returns:
            True: 到達成功 / False: 失敗またはキャンセル
        """
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.info("NavigateToPose action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped(
            header=Header(
                frame_id="map",
                stamp=self.parent_node.get_clock().now().to_msg(),
            ),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2),
                    w=math.cos(yaw / 2),
                ),
            ),
        )

        self.info(f"Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})...")
        send_goal_future = self._nav_client.send_goal_async(goal_msg, self._feedback_callback)
        rclpy.spin_until_future_complete(self.parent_node, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f"Goal to ({x:.2f}, {y:.2f}) was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()

        while not self._is_nav_complete():
            rclpy.spin_once(self.parent_node, timeout_sec=0.1)

        return self.status == GoalStatus.STATUS_SUCCEEDED

    def cancel_nav(self):
        """現在のナビゲーションをキャンセルする。"""
        self.info("Canceling current goal.")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.parent_node, future)

    def get_current_pose_stamped(self) -> PoseStamped | None:
        """現在のロボット姿勢を PoseStamped 形式で返す。

        Returns:
            PoseStamped: map座標系での現在姿勢 / None: 未取得の場合
        """
        if self.current_pose is None:
            self.warn("Current pose is not available yet.")
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.parent_node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = self.current_pose
        return pose_stamped

    # ── 内部メソッド ──────────────────────────────────────────────

    def _is_nav_complete(self) -> bool:
        if not self.result_future:
            return True
        rclpy.spin_until_future_complete(self.parent_node, self.result_future, timeout_sec=0.1)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.warn(f"Navigation failed with status: {self.status}")
            return True
        return False

    def _wait_for_node_to_activate(self, node_name: str):
        self.info(f"Waiting for {node_name} to become active...")
        state_client = self.parent_node.create_client(GetState, f"{node_name}/get_state")
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f"{node_name}/get_state service not available, waiting...")
        req = GetState.Request()
        while True:
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self.parent_node, future)
            if future.result() is not None and future.result().current_state.label == "active":
                break
            time.sleep(2)

    def _wait_for_initial_pose(self):
        while not self.initial_pose_received:
            self.info("Waiting for initial pose...")
            self._publish_initial_pose()
            rclpy.spin_once(self.parent_node, timeout_sec=1)

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = "map"
        msg.header.stamp = self.parent_node.get_clock().now().to_msg()
        self._initial_pose_pub.publish(msg)

    def _tf_callback(self, msg: TFMessage):
        self.initial_pose_received = True
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.current_pose = Pose(
                    position=transform.transform.translation,
                    orientation=transform.transform.rotation,
                )
                break

    def _feedback_callback(self, msg):
        pass  # ナビゲーション中のフィードバックを使う場合はここで処理する

    # ── ロガーのショートカット ────────────────────────────────────

    def info(self, msg: str):
        self.parent_node.get_logger().info(msg)

    def warn(self, msg: str):
        self.parent_node.get_logger().warn(msg)

    def error(self, msg: str):
        self.parent_node.get_logger().error(msg)
