#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

NUM_WAYPOINT = 3
DIST_TOLERANCE = 0.3

class TurtleController(Node):

    def __init__(self):
        super().__init__("turtle_controller")

        # Publisher: sends velocity commands to the turtle
        self.publisher_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Subscriber: receives the turtle position
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        # Timer to publish commands periodically
        self.timer = self.create_timer(0.1, self.control_loop)

        # Store current pose
        self.pose = None

        # Create Waypoints
        self.create_list_waypoint()
        
        #Aux to know what is the current objective
        self.current_waypoint = 0

        #Used to know if all the waypoints were visited
        self.done = False
    
    def create_list_waypoint(self):
        self.waypoint = [
            (
                random.uniform(1,10), # x
                random.uniform(1,10), # y
            ) for _ in range (NUM_WAYPOINT)
        ]
        self.get_logger().info(f"Defined waypoints:")
        for i, wp in enumerate(self.waypoint):
            self.get_logger().info(f"  WP[{i}]: x={wp[0]:.2f}, y={wp[1]:.2f}")
      

    def pose_callback(self, msg):
        """Callback function for subscriber"""
        self.pose = msg


    def control_loop(self):
        """Main control loop (publisher)"""
        if self.pose is None or self.done:
            return  # wait until we receive the first pose or all the waypoints were already visited

        if self.current_waypoint < len(self.waypoint):

            dx_goal, dy_goal = self.waypoint[self.current_waypoint]
            dx = dx_goal - self.pose.x
            dy = dy_goal - self.pose.y
            dist_error = math.sqrt(dx**2 + dy**2) 

            
            if dist_error < DIST_TOLERANCE :
                self.get_logger().info( f"Objective ({dx_goal,dy_goal}) achived\n")
                self.current_waypoint += 1
                return
             
            msg = Twist()
            theta_goal = math.atan2(dy,dx)
            theta_error = theta_goal - self.pose.theta
            if(abs(theta_error) > 0.1):
                msg.angular.z = 1.0 * theta_error #Limited velocity for better visualization
                msg.linear.x = 0.0
            else:
                msg.angular.z = 0.0
                msg.linear.x = 1.5 * dist_error 

            self.publisher_.publish(msg)  
        else:
            self.done = True
            self.publisher_.publish(Twist()) #Stop turtle
            self.get_logger().info("Achived all points!")    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()