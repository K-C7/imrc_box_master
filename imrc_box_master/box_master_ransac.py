import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import math
from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from imrc_messages.action import BoxCommand
from imrc_messages.msg import GeneralCommand
from imrc_messages.msg import RobotActionProgress
from imrc_messages.msg import WallInfo

class Direction(Enum):
    FRONT = 1
    BACK = 2
    LEFT = 3
    RIGHT = 4


class BoxMaster(Node):
    def __init__(self):
        super().__init__('box_manager')
        self.logger = self.get_logger()

        # 並行処理を許可するグループを作成
        self.cb_group = ReentrantCallbackGroup()

        # パブリッシャー
        self.gc_pub = self.create_publisher(GeneralCommand, '/robot_command', 10, callback_group=self.cb_group)

        self.debug = self.declare_parameter('debug', False)
        if(self.debug.value == True):
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_uart', 10, callback_group=self.cb_group)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_box', 10, callback_group=self.cb_group)

        # サブスクライバー
        self.lift_progress = "IDLE"
        self.complete_sub = self.create_subscription(
            RobotActionProgress, '/robot_progress', self.lift_progress_callback, 10, callback_group=self.cb_group)
        
        self.wall_sub = self.create_subscription(WallInfo, "/wall", self.wall_callback, 10, callback_group=self.cb_group)
        self.wall_sub = self.create_subscription(WallInfo, "/wall_raw", self.wall_raw_callback, 10, callback_group=self.cb_group)

        # -------------------- 各種フラグ・パラメータ --------------------
        self.DISTANCE_BOX = 0.325
        self.DISTANCE_WALL_SIDE_SHORT = 1.180
        self.DISTANCE_WALL_SIDE_LONG = 1.712

        self.source_direction = Direction.BACK
        self.distance = 0.0
        self.alignment_enable = False
        self.isMovingForward = False
        self.target_dist = self.DISTANCE_WALL_SIDE_SHORT
        self.kp_align = 0.80
        self.max_speed = 0.20
        self.dead_zone = 0.03

        self.rotate_enable = False
        self.kp_rotate = 0.3
        self.tolerance = 0.010
        self.max_angular_vel = 0.3

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

        if(self.target_dist == self.DISTANCE_WALL_SIDE_LONG):
            return

        distance = 0.00
        angle = 0.00

        
        if(self.source_direction == Direction.FRONT):
            distance = msg.front_distance
            angle = msg.front_angle
        elif(self.source_direction == Direction.BACK):
            distance = msg.back_distance
            angle = msg.back_angle - 0.040
        elif(self.source_direction == Direction.LEFT):
            distance = msg.left_distance
            angle = msg.left_angle
        elif(self.source_direction == Direction.RIGHT):
            distance = msg.right_distance
            angle = msg.right_angle
        else:
            pass

        if self.alignment_enable:
            self.alignment_callback(distance)
        elif self.rotate_enable:
            self.rotate_callback(angle)
        elif self.isMovingForward:
            pass
        else:
            self.stop_robot()
    
    def wall_raw_callback(self, msg): 
        # ノード終了時は何もしない
        if not rclpy.ok():
            return

        if not(self.target_dist == self.DISTANCE_WALL_SIDE_LONG):
            return

        distance = 0.00
        angle = 0.00

        
        if(self.source_direction == Direction.FRONT):
            distance = msg.front_distance
            angle = msg.front_angle
        elif(self.source_direction == Direction.BACK):
            distance = msg.back_distance
            angle = msg.back_angle
        elif(self.source_direction == Direction.LEFT):
            distance = msg.left_distance
            angle = msg.left_angle
        elif(self.source_direction == Direction.RIGHT):
            distance = msg.right_distance
            angle = msg.right_angle
        else:
            pass

        if self.alignment_enable:
            self.alignment_callback(distance)
        elif self.rotate_enable:
            self.rotate_callback(angle)
        elif self.isMovingForward:
            pass
        else:
            self.stop_robot()
    
    def rotate_callback(self, error):
        error = -error
        twist = Twist()
        if abs(error) > self.tolerance:
            speed = self.kp_rotate * error
            speed = max(min(speed, self.max_angular_vel), -self.max_angular_vel)
            twist.angular.z = speed
            self.get_logger().debug('Rotating now. {0}'.format(error))
        else:
            twist.angular.z = 0.0
            self.rotate_enable = False
            self.get_logger().info('Rotation complete!')
        self.cmd_vel_pub.publish(twist)
    
    def alignment_callback(self, distance):
        error = distance  - self.target_dist
        twist = Twist()
        if abs(error) > self.dead_zone:
            self.get_logger().debug('Alignment now. {0}'.format(error))
            speed = self.kp_align * error
            speed = max(min(speed, self.max_speed), -self.max_speed)
            if(self.source_direction == Direction.FRONT):
                twist.linear.x = speed
            elif(self.source_direction == Direction.BACK):
                twist.linear.x = -speed
            elif(self.source_direction == Direction.LEFT):
                twist.linear.y = speed
            elif(self.source_direction == Direction.RIGHT):
                twist.linear.y = -speed
        else:
            twist.linear.x = 0.0
            self.alignment_enable = False
            self.get_logger().info('Target distance reached.')
        self.cmd_vel_pub.publish(twist)

    # ------------------------- アクション実行メイン -------------------------

    async def box_command_callback(self, goal_handle):
        
        if not (goal_handle.request.command == "LONG" or goal_handle.request.command == "SHORT"):
            self.get_logger().error(f"Action aborted because of invalid command. Command: {goal_handle.request.command}")
            goal_handle.abort()
            return BoxCommand.Result()
        
        self.get_logger().info(f"Action accepted. Command: {goal_handle.request.command}")

        # 1. 指定角度への回転
        self.source_direction = Direction.BACK
        self.target_dist = self.DISTANCE_BOX
        self.rotate_enable = True
        if not self.wait_for_flag(lambda: self.rotate_enable, False, goal_handle):
            return self.handle_exit(goal_handle)
        
        # 2. 横壁との距離合わせ
        self.source_direction = Direction.RIGHT
        if(goal_handle.request.command == "LONG"):
            self.target_dist = self.DISTANCE_WALL_SIDE_LONG
        else:
            self.target_dist = self.DISTANCE_WALL_SIDE_SHORT

        self.alignment_enable = True
        if not self.wait_for_flag(lambda: self.alignment_enable, False, goal_handle):
            return self.handle_exit(goal_handle)
        
        # 2. ダンボールとの距離合わせ
        self.source_direction = Direction.BACK
        self.target_dist = self.DISTANCE_BOX
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
        self.get_logger().info("Moving forward...")
        twist = Twist()
        twist.linear.x = 0.2
        rate = self.create_rate(10)
        self.isMovingForward = True
        for _ in range(10):
            if not rclpy.ok() or goal_handle.is_cancel_requested:
                return self.handle_exit(goal_handle)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        self.isMovingForward = False
        self.stop_robot()
        self.get_logger().info("Box action done.")
        
        # 成功終了
        result = BoxCommand.Result()
        result.result = self.lift_progress
        self.lift_progress = "IDLE"
        goal_handle.succeed()
        return result

    def wait_for_flag(self, get_flag_func, target_value, goal_handle):
        """フラグ待ちループ。ROS終了・キャンセル・目標達成のいずれかで抜ける"""
        while rclpy.ok() and get_flag_func() != target_value:
            rate = self.create_rate(100)
            if goal_handle.is_cancel_requested:
                return False
            rate.sleep()
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