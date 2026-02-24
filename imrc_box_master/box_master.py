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
        self.complete_sub = self.create_subscription(RobotActionProgress, '/robot_progress', self.lift_progress_callback, 10)

        self.action_server = ActionServer(
            self,
            BoxCommand,
            "box_master",
            self.box_command_callback,
        )
    
    def lift_progress_callback(self, msg):
        if(msg.target == "ball"):
            self.lift_progress = msg.state

    
    def box_command_callback(self, goal_handle):
        gc = GeneralCommand()
        gc.target = "ball"
        if(goal_handle.request.command == "drop"):
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


