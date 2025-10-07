#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist


class Publisher(Node):
    def __init__(self) -> None:
        # initialize base class (must happen before everything else)
        super().__init__("publisher")
				
        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            
        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)

        self.motor_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)



    def hb_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 1.
        msg.angular.z = 1.
        msg.angular.y = 5.

       # publish heartbeat counter
        self.hb_pub.publish(msg)

    def kill_callback(self, msg: Bool) -> None:
        if msg.data:
            self.hb_timer.cancel()
            twist = Twist()
            twist.linear.x = 0.
            twist.angular.z = 0.
            twist.angular.y = 0.

        # publish heartbeat counter
            self.hb_pub.publish(twist)




if __name__ == "__main__":
    rclpy.init()
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()
    