import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import DOGZILLALib as dog
import time
from rclpy.duration import Duration
import sys
from Speech_Lib import Speech


class DogAction(Node):
   def __init__(self):
        super().__init__("dog_action")
        self.subscription = self.create_subscription(String,'dog_action',self.action_callback,10)
        self.subscription
        self.dogControl = dog.DOGZILLA()
        #self.spe = Speech()

   def action_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        if msg.data == "lie":
            self.dogControl.action(1)
            print("action 1")
        elif msg.data == "swing":
            self.dogControl.action(16)
            print("action 16")
        elif msg.data == "pee":
            self.dogControl.action(11)
            print("action 11")
        elif msg.data == "restore":
            self.dogControl.action(255)
            print("restore!!!")



def main(args=None):
    rclpy.init(args=args)
    node = DogAction()
    rclpy.spin(node)
    rclpy.shutdown()

