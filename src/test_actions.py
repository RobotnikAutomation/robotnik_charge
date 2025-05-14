import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from robotnik_navigation_msgs.action import Dock
from robotnik_navigation_msgs.action import Move


class ActionServerTest(Node):

    def __init__(self):
        super().__init__('test_action_server')
        self.dock_action_server = ActionServer(
            self,
            Dock,
            'dock',
            self.dock_execute_callback)
        self.move_action_server = ActionServer(
            self,
            Move,
            'move',
            self.move_execute_callback)

    def dock_execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Dock.Result()
        feedback_msg = Dock.Feedback()
        i = 0
        while i<10:
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
            i+=1
        goal_handle.succeed()
        return result
    
    def move_execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Move.Result()
        feedback_msg = Move.Feedback()
        i = 0
        while i<10:
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
            i+=1
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    test_action_server = ActionServerTest()

    rclpy.spin(test_action_server)


if __name__ == '__main__':
    main()