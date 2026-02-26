import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
from imrc_messages.msg import WallInfo


class BoxMaster(Node):
    def __init__(self):
        super().__init__('box_manager')
        self.logger = self.get_logger()

        # 並行処理を許可するグループを作成
        self.cb_group = ReentrantCallbackGroup()

        # パブリッシャー
        self.gc_pub = self.create_publisher(GeneralCommand, '/robot_command', 10, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_box', 10, callback_group=self.cb_group)

        # サブスクライバー
        self.lift_progress = "IDLE"
        self.complete_sub = self.create_subscription(
            RobotActionProgress, '/robot_progress', self.lift_progress_callback, 10, callback_group=self.cb_group)
        
        self.wall_sub = self.create_subscription(WallInfo, "/wall", self.wall_callback, 10, callback_group=self.cb_group)

        # -------------------- 各種フラグ・パラメータ --------------------
        self.distance = 0.0
        self.alignment_enable = False
        self.target_dist = 0.23
        self.kp_align = 0.3
        self.max_speed = 0.10
        self.dead_zone = 0.01

        self.rotate_enable = False
        self.kp_rotate = 1.5
        self.tolerance = 0.02
        self.max_angular_vel = 0.2

        # -------------------- アクションサーバー --------------------
        self.action_server = ActionServer(
            self,
            BoxCommand,
            "box_master",
            execute_callback=self.box_command_callback,
            callback_group=self.cb_group,
            cancel_callback=self.cancel_callback, # キャンセルを許可
            goal_callback=self.goal_callback      # ゴール受付を許可
        )

    # ------------------------- アクション制御コールバック -------------------------
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def stop_robot(self):
        # 終了処理中にパブリッシュしようとしてエラーになるのを防ぐ
        if not rclpy.ok():
            return
        msg = Twist()
        try:
            self.cmd_vel_pub.publish(msg)
        except Exception:
            pass

    # ------------------------- コールバック群 -------------------------

    def lift_progress_callback(self, msg):
        if msg.target == "ball" and msg.param == "store":
            self.get_logger().info(f"Received robot progress: {msg.state}")
            self.lift_progress = msg.state
        else:
            self.lift_progress = "IDLE"

    def wall_callback(self, msg): 
        # ノード終了時は何もしない
        if not rclpy.ok():
            return

        if self.alignment_enable:
            self.alignment_callback(msg)
        elif self.rotate_enable:
            self.rotate_callback(msg)
        else:
            self.stop_robot()
    
    def rotate_callback(self, msg):
        error = msg.back_degree
        twist = Twist()
        if abs(error) > self.tolerance:
            speed = self.kp_rotate * error
            speed = max(min(speed, self.max_angular_vel), -self.max_angular_vel)
            twist.angular.z = speed
        else:
            twist.angular.z = 0.0
            self.rotate_enable = False
            self.get_logger().info('Rotation complete!')
        self.cmd_vel_pub.publish(twist)
    
    def alignment_callback(self, msg):
        error = msg.back_distance
        twist = Twist()
        if abs(error) > self.dead_zone:
            speed = self.kp_align * error
            speed = max(min(speed, self.max_speed), -self.max_speed)
            twist.linear.x = speed
        else:
            twist.linear.x = 0.0
            self.alignment_enable = False
            self.get_logger().info('Target distance reached.')
        self.cmd_vel_pub.publish(twist)

    # ------------------------- アクション実行メイン -------------------------

    async def box_command_callback(self, goal_handle):
        self.get_logger().info(f"Action accepted. Target yaw: {goal_handle.request.command}")
        
        try:
            # ここでは目標角度を受け取るだけで、実際の回転はrotate_callbackで行う構成と推測
            # self.target_yaw は wall_callback 側で参照されるべき
            pass 
        except ValueError:
            goal_handle.abort()
            return BoxCommand.Result()
        
        # 1. 指定角度への回転
        self.rotate_enable = True
        if not self.wait_for_flag(lambda: self.rotate_enable, False, goal_handle):
            return self.handle_exit(goal_handle)
        
        # 2. 距離合わせ
        self.alignment_enable = True
        if not self.wait_for_flag(lambda: self.alignment_enable, False, goal_handle):
            return self.handle_exit(goal_handle)

        # 3. ボール取得コマンド送信
        gc = GeneralCommand()
        gc.target = "ball"
        gc.param = 2
        self.gc_pub.publish(gc)

        # 4. リフト完了待ち
        if not self.wait_for_flag(lambda: self.lift_progress, 'OK', goal_handle):
            return self.handle_exit(goal_handle)

        # 5. 少し前進
        self.get_logger().info("Moving backward...")
        twist = Twist()
        twist.linear.x = 0.1
        for _ in range(10):
            if not rclpy.ok() or goal_handle.is_cancel_requested:
                return self.handle_exit(goal_handle)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        self.stop_robot()
        
        # 成功終了
        result = BoxCommand.Result()
        result.result = self.lift_progress
        self.lift_progress = "IDLE"
        goal_handle.succeed()
        return result

    def wait_for_flag(self, get_flag_func, target_value, goal_handle):
        """フラグ待ちループ。ROS終了・キャンセル・目標達成のいずれかで抜ける"""
        while rclpy.ok() and get_flag_func() != target_value:
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(0.1)
        # 目標に達しており、かつROSが正常ならTrue、それ以外（キャンセル等）はFalse
        return rclpy.ok() and not goal_handle.is_cancel_requested

    def handle_exit(self, goal_handle):
        """中断時のクリーンアップ処理"""
        self.rotate_enable = False
        self.alignment_enable = False
        self.stop_robot()
        
        result = BoxCommand.Result()
        result.result = "ABORTED_OR_CANCELED"

        if not rclpy.ok():
            return result

        if goal_handle.is_cancel_requested:
            self.get_logger().warn("Action has been canceled.")
            goal_handle.canceled()
        else:
            self.get_logger().error("Action failed or was interrupted.")
            goal_handle.abort()
        
        return result


def main(args=None):
    rclpy.init(args=args)
    box_master = BoxMaster()

    executor = MultiThreadedExecutor()
    executor.add_node(box_master)

    try:
        executor.spin()
    except KeyboardInterrupt:
        box_master.get_logger().info('KeyboardInterrupt received.')
    finally:
        # シャットダウン手順:
        # 1. まずコンテキストをシャットダウンして、rclpy.ok()をFalseにする
        #    これにより、別スレッドのwhileループが即座に終了する
        if rclpy.ok():
            rclpy.shutdown()
        
        # 2. エグゼキュータとノードを掃除
        executor.shutdown()
        box_master.destroy_node()


if __name__ == '__main__':
    main()