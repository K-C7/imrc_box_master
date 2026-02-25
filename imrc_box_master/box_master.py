import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from imrc_messages.action import BoxCommand
from imrc_messages.msg import GeneralCommand
from imrc_messages.msg import RobotActionProgress


class BoxMaster(Node):
    def __init__(self):
        super().__init__('box_manager')
        self.logger = self.get_logger()

        # 並行処理を許可するグループを作成
        self.cb_group = ReentrantCallbackGroup()

        # パブリッシャー
        self.gc_pub = self.create_publisher(GeneralCommand, '/robot_command', 10, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_box', 10, callback_group=self.cb_group)

        # サブスクライバー (callback_groupを指定して並行動作させる)
        self.lift_progress = "IDLE"
        self.complete_sub = self.create_subscription(
            RobotActionProgress, '/robot_progress', self.lift_progress_callback, 10, callback_group=self.cb_group)

        self.depth_sub = self.create_subscription(
            String, '/depth_status', self.depth_callback, 10, callback_group=self.cb_group)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)

        # -------------------- 各種フラグ・パラメータ --------------------
        self.distance = 0.0
        self.alignment_enable = False
        self.target_dist = 0.23
        self.kp_align = 0.3
        self.max_speed = 0.10
        self.dead_zone = 0.01

        self.current_yaw = 0.0
        self.start_yaw = None
        self.is_rotating = False
        self.rotate_enable = False
        self.target_yaw = 0.0
        self.kp_rotate = 1.5
        self.tolerance = 0.02
        self.max_angular_vel = 0.2

        # -------------------- アクションサーバー --------------------
        self.action_server = ActionServer(
            self,
            BoxCommand,
            "box_master",
            execute_callback=self.box_command_callback,
            callback_group=self.cb_group # ここが重要
        )

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    # ------------------------- コールバック群 -------------------------

    def lift_progress_callback(self, msg):
        if msg.target == "ball" and msg.param == "store":
            self.lift_progress = msg.state
        else:
            self.lift_progress = "IDLE"

    def depth_callback(self, msg):
        if not self.alignment_enable:
            return

        # 文字列が数値かどうかチェック
        data_str = msg.data.replace(".", "")
        if data_str.isdigit() or (data_str.startswith('-') and data_str[1:].isdigit()):
            self.distance = float(msg.data) * 0.01
        else:
            self.get_logger().warn("Can't read depth sensor data! Backing up.")
            twist = Twist()
            twist.linear.x = -0.05
            self.cmd_vel_pub.publish(twist)
            return
        
        error = self.distance - self.target_dist
        twist = Twist()

        if abs(error) > self.dead_zone:
            speed = self.kp_align * error
            speed = max(min(speed, self.max_speed), -self.max_speed)
            twist.linear.x = speed
        else:
            twist.linear.x = 0.0
            self.alignment_enable = False # 完了
            self.get_logger().info(f'Target distance reached: {self.distance:.2f}')
        
        self.cmd_vel_pub.publish(twist)

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        
        if not self.rotate_enable:
            return

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.is_rotating = True
            self.get_logger().info(f'Start rotation to: {math.degrees(self.target_yaw):.2f} deg')
            return

        # 制御計算
        error = self.target_yaw - self.current_yaw
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi

        twist = Twist()
        if abs(error) > self.tolerance:
            speed = self.kp_rotate * error
            speed = max(min(speed, self.max_angular_vel), -self.max_angular_vel)
            twist.angular.z = speed
        else:
            twist.angular.z = 0.0
            self.start_yaw = None
            self.is_rotating = False
            self.rotate_enable = False # 完了
            self.get_logger().info('Rotation complete!')
        
        self.cmd_vel_pub.publish(twist)

    # ------------------------- アクション実行メイン -------------------------

    async def box_command_callback(self, goal_handle):
        self.get_logger().info(f"Action accepted. Target yaw: {goal_handle.request.command}")
        
        try:
            self.target_yaw = float(goal_handle.request.command)
        except ValueError:
            self.get_logger().error("Invalid command. Not a number.")
            goal_handle.abort()
            return BoxCommand.Result()

        # 1. 指定角度への回転
        self.rotate_enable = True
        if not self.wait_for_flag(lambda: self.rotate_enable, False, goal_handle):
            return self.cancel_action(goal_handle)

        # 2. 距離合わせ
        self.alignment_enable = True
        if not self.wait_for_flag(lambda: self.alignment_enable, False, goal_handle):
            return self.cancel_action(goal_handle)

        # 3. 180度回転 (π加算)
        self.target_yaw += math.pi
        while self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
        while self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi
        
        self.rotate_enable = True
        if not self.wait_for_flag(lambda: self.rotate_enable, False, goal_handle):
            return self.cancel_action(goal_handle)

        # 4. ボール取得コマンド送信
        gc = GeneralCommand()
        gc.target = "ball"
        gc.param = 2
        self.gc_pub.publish(gc)

        # 5. リフト完了待ち
        if not self.wait_for_flag(lambda: self.lift_progress, 'OK', goal_handle):
            return self.cancel_action(goal_handle)

        # 6. 少し前進
        self.get_logger().info("Moving forward...")
        twist = Twist()
        twist.linear.x = 0.1
        for _ in range(10): # 1.0秒間 (0.1s * 10)
            if goal_handle.is_cancel_requested:
                return self.cancel_action(goal_handle)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop_robot()
        
        # 結果を返して成功終了
        result = BoxCommand.Result()
        result.result = self.lift_progress
        self.lift_progress = "IDLE"
        
        goal_handle.succeed()
        return result

    def wait_for_flag(self, get_flag_func, target_value, goal_handle):
        """フラグが指定の値になるまで待機するヘルパー（キャンセル対応）"""
        while get_flag_func() != target_value:
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(0.1)
        return True

    def cancel_action(self, goal_handle):
        """キャンセル時の共通処理"""
        self.stop_robot()
        self.rotate_enable = False
        self.alignment_enable = False
        goal_handle.canceled()
        return BoxCommand.Result()


def main(args=None):
    rclpy.init(args=args)
    box_master = BoxMaster()

    # MultiThreadedExecutorを使用して並列実行を可能にする
    executor = MultiThreadedExecutor()
    executor.add_node(box_master)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        box_master.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()