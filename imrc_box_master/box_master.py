import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

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

        self.gc_pub = self.create_publisher(GeneralCommand, '/robot_command', 10)
        self.lift_progress = "IDLE"
        self.complete_sub = self.create_subscription(RobotActionProgress, '/robot_progress', self.lift_progress_callback, 10)

        # -------------------- 位置合わせ config --------------------
        self.depth_sub = self.create_subscription(String, '/depth_status', self.depth_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_uart', 10)
        self.distance = 0.0
        self.alignment_enable = False
        
        self.target_dist = 0.37   # 目標距離 (m)
        self.kp = 1.0            # 比例ゲイン (速度の強さ)
        self.max_speed = 0.2     # 最大速度 (m/s) 制限用
        self.dead_zone = 0.02    # 不感帯 (cm以内の誤差は無視して停止)
        # -------------------- 位置合わせ config --------------------


        # -------------------- 回転 config --------------------
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 状態管理用変数
        self.current_yaw = None
        self.start_yaw = None
        self.target_yaw = None
        self.is_rotating = False

        self.rotate_enable = False

        # 制御パラメータ
        self.kp = 1.5           # 回転速度の比例ゲイン
        self.tolerance = 0.02   # 許容誤差 (ラジアン、約1度)
        self.max_angular_vel = 0.2 # 最大回転速度 (rad/s)

        # -------------------- 回転 config --------------------

        self.action_server = ActionServer(
            self,
            BoxCommand,
            "box_master",
            self.box_command_callback,
        )

    
    # ------------------------- Depthで位置合わせ -------------------------
    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def depth_callback(self, msg):
        if(self.rotate_enable == False and self.alignment_enable == False):
            self.stop_robot()

        # alignment_enableがTrueのときだけロボットを動かす
        if(self.alignment_enable == False):
            self.stop_robot()
            return

        if(str(msg.data).replace(".","").isdigit()):
            self.distance = float(msg.data) * 0.01
        else:
            # self.distance = 0.0
            self.stop_robot()
            self.get_logger().warn("Can't read depth sensor data! Stopping Robot.")
            return
        
        d = self.distance
        error = d - self.target_dist

        twist = Twist()
        if(abs(error) > self.dead_zone):
            speed = self.kp * error
            if(speed > self.max_speed):
                speed = self.max_speed
            elif(speed < -self.max_speed):
                speed = -self.max_speed
            
            twist.linear.x = speed
            self.get_logger().info(f'Dist: {d:.2f}, Speed: {speed:.2f}m/s')
        else:
            twist.linear.x = 0.0
            self.get_logger().info(f'Target distance reached. Dist: {d:.2f}')
            self.alignment_enable = False
        
        self.cmd_vel_pub.publish(twist)
    

    # ------------------------- 180度回転 -------------------------
    def euler_from_quaternion(self, quaternion):
        """クォータニオンからYaw角を計算"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        if(self.rotate_enable == False and self.alignment_enable == False):
            self.stop_robot()

        if(self.rotate_enable == False):
            return

        # 現在のYaw角を更新
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        # 最初の1回だけ、目標角度を設定
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            # 180度 (π) 加算
            self.target_yaw = self.start_yaw + math.pi
            
            # 角度を -π から π の範囲に正規化
            if self.target_yaw > math.pi:
                self.target_yaw -= 2 * math.pi
            elif self.target_yaw < -math.pi:
                self.target_yaw += 2 * math.pi
                
            self.is_rotating = True
            self.get_logger().info(f'Start rotation! Target orientation is: {math.degrees(self.target_yaw):.2f} rad')
            return

        if self.is_rotating:
            self.control_loop()

    def control_loop(self):
        # 目標までの誤差を計算
        error = self.target_yaw - self.current_yaw
        
        # 誤差も -π ～ π に正規化 (最短距離で回るため)
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi

        twist = Twist()

        if abs(error) > self.tolerance:
            # P制御による回転速度の決定
            speed = self.kp * error
            
            # 速度制限
            if speed > self.max_angular_vel: speed = self.max_angular_vel
            if speed < -self.max_angular_vel: speed = -self.max_angular_vel
            
            twist.angular.z = speed
            self.cmd_vel_pub.publish(twist)
        else:
            # 停止
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.is_rotating = False
            self.rotate_enable = False
            self.get_logger().info('Rotation complete!')


    # ------------------------- 上昇、アーム制御 -------------------------

    def lift_progress_callback(self, msg):
        if(msg.target == "ball"):
            self.lift_progress = msg.state
    
    
    def box_command_callback(self, goal_handle):
        gc = GeneralCommand()
        gc.target = "ball"
        if(goal_handle.request.command == "ready"):
            gc.param = 1
        elif(goal_handle.request.command == "drop"):
            self.alignment_enable = True
            while(self.alignment_enable == True):
                rclpy.spin_once(self)

            self.rotate_enable = True
            while(self.rotate_enable == True):
                rclpy.spin_once(self)
            gc.param = 2
        else:
            gc.param = 1
        
        self.gc_pub.publish(gc)

        while(self.lift_progress != 'OK'):
            rclpy.spin_once(self)
        
        result = BoxCommand.Result()
        result.result = self.lift_progress

        self.lift_progress = "IDLE"

        goal_handle.succeed()
        return result
    


def main(args = None):
    rclpy.init(args=args)
    box_master = BoxMaster()
    rclpy.spin(box_master)

    box_master.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


