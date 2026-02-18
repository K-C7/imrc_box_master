import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from imrc_messages.action import BoxCommand
from imrc_messages.msg import GeneralCommand
from imrc_messages.msg import RobotActionProgress

class BoxMaster(Node):
    def __init__(self):
        super().__init__('box_manager')
        self.logger = self.get_logger()

        self.gc_pub = self.create_publisher(GeneralCommand, '/robot_command', 10)
        self.lift_progress = "IDLE"
        self.complete_sub = self.create_subscription(RobotActionProgress, '/robot_progress', 10)

        self.action_server = ActionServer(
            self,
            BoxCommand,
            self.box_command_callback
        )
    
    def lift_progress_callback(self, msg):
        if(msg.target == "lift"):
            self.lift_progress = msg.state

    
    def box_command_callback(self, goal_handle):
        gc = GeneralCommand()
        gc.target = "lift"
        if(goal_handle.request.command == "up"):
            gc.param = 1
        else:
            gc.param = 0
        
        self.gc_pub.publish(gc)

        while(self.lift_progress != 'OK'):
            pass

        goal_handle.succeed()
        return self.lift_progress

def main(args = None):
    rclpy.init(args=args)
    uart_bridge = BoxMaster()
    rclpy.spin(uart_bridge)

    uart_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


