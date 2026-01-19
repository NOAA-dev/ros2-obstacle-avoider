#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
 
 
class ObjectAvoiderNode(Node): 
    def __init__(self):
        super().__init__("object_avoider")
        self.t = float()
        self.declare_parameter("min_distance",1.2)
        self.declare_parameter("forward_speed",1.0)
        self.min_distance = self.get_parameter("min_distance").value  # meters
        self.forward_speed = self.get_parameter("forward_speed").value  # meters/second

        self.subscriber_ = self.create_subscription(LaserScan,"/scan",self.laser_callback,10,)
        self.publisher_  = self.create_publisher(Twist,"/cmd_vel",10)
 


    def laser_callback(self,msg= LaserScan):
        # --- Raw fields ---
        angle_min = msg.angle_min        # rad
        angle_max = msg.angle_max        # rad
        angle_inc = msg.angle_increment  # rad
        ranges = msg.ranges              # list[float]
        intensities = msg.intensities    # list[float] (may be empty)

        front_range = []
        right_range = []
        left_range  = []

        angle = angle_min
        for i, r in enumerate(ranges):
            if math.isfinite(r):  # ignore inf / nan
                if(angle >= -2.094 and angle <= -0.523598776 ):    
                    right_range.append(r)
                if(angle >= -0.523598776 and angle <= 0.523598776 ):
                    front_range.append(r)
                if(angle >= 0.523598776 and angle <= 2.094 ):
                    left_range.append(r)
            angle += angle_inc

        front_min = self.safe_check(front_range)
        left_min  = self.safe_check(left_range)
        right_min = self.safe_check(right_range)
        # self.get_logger().info(f"Front Min: {front_min} , Left Min: {left_min} , Right Min: {right_min}")

        msg = Twist()

        if(front_min < self.min_distance):
            error = left_min - right_min
            if abs(error) > 1.0:
                msg.linear.x = 0.0
                msg.angular.z = min(0.6 * error, 1.0) if error > 0 else max(0.6 * error, -1.0)
                self.publisher_.publish(msg)
            else:
                msg.linear.x = 0.05
                msg.angular.z = -0.6
                self.publisher_.publish(msg)
        else:
            msg.linear.x = self.forward_speed
            msg.angular.z = 0.0
            self.publisher_.publish(msg)


    def safe_check(self, arr):
        return min(arr) if arr else float('inf')
            
    # def turn_right(self):
    #      msg = Twist()
    #      msg.linear.x = 0.0
    #      msg.angular.z = -0.3
    #      self.publisher_.publish(msg)

    # def turn_left(self):
    #      msg = Twist()
    #      msg.linear.x = 0.0
    #      msg.angular.z = 0.3
    #      self.publisher_.publish(msg)

    # def u_turn(self):
    #      msg = Twist()
    #      msg.linear.x = 0.0
    #      msg.angular.z = -0.5
    #      self.publisher_.publish(msg)

     
def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoiderNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
