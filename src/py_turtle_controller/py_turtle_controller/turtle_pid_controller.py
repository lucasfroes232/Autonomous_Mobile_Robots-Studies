#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist 
import math
import random
import time

N_WAYPOINTS = 3
DESORIENTATION_TOLERANCE = 0.1
DIST_TOLERANCE = 0.3

class PID_TurtleController(Node):

    #Construtor
    def __init__(self):

        super().__init__("PID_turtle_controller")

        #Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        #Subscriber
        self.subscription_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        #Original pose
        self.pose = None 

        #Timer to send commands
        self.timer = self.create_timer(0.1,self.PID_control_loop)
        

        #create waypoints
        self.createWayPoints()

        #current waypoint
        self.currentWp = 0

        # Sum of last errors
        self.I_theta = 0.0
        self.I_dist = 0.0

        # Rate of change
        self.D_theta = 0.0
        self.D_dist = 0.0

        # Past error
        self.past_dist_error = 0.0
        self.past_theta_error = 0.0

        #time
        self.last_time = time.monotonic()

        #Check if all the wp were visited
        self.isDone = False



    def createWayPoints(self):
        self.waypoints = [
            (
                random.uniform(0.5,10),
                random.uniform(0.5,10)
            )for _ in range(N_WAYPOINTS)
        ]
        self.get_logger().info(
            f"{N_WAYPOINTS} Defined Points:\n"
        )
        for i,wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"Point:{i}; x:{wp[0]:.2f}, y:{wp[1]:.2f}"
            )


    
    def pose_callback(self,msg):
        self.pose = msg

    def PID_control_loop(self,Kp=1.0,Ki=0.1,Kd=0.1):

        if self.pose is None or self.isDone :
            return
        
        if self.currentWp < len(self.waypoints):
            x_goal, y_goal = self.waypoints[self.currentWp]
            dx = x_goal - self.pose.x
            dy = y_goal - self.pose.y
            error_dist = math.sqrt(dx**2 + dy**2)
            
            current_time = time.monotonic()

            dt = time.monotonic() - self.last_time
            if dt<= 0:
                return

            #Incrementing D_dist
            self.D_dist = (error_dist - self.past_dist_error)/ dt # delta Dist/ delta time
        
             #Incrementing I_dist
            self.I_dist += error_dist * dt

            if error_dist < DIST_TOLERANCE:
                self.get_logger().info( f"Objective ({x_goal,y_goal}) achived\n")
                self.currentWp+= 1
                return
            
            msg = Twist()
            theta_goal = math.atan2(dy,dx)
            theta_error = theta_goal - self.pose.theta
            #Incrementing D_theta
            self.D_theta = (theta_error - self.past_theta_error)/ dt

            #Incrementing I_theta
            self.I_theta += theta_error * dt

            
            
            if abs(theta_error) > DESORIENTATION_TOLERANCE:
                msg.angular.z = Kp * theta_error + Ki * self.I_theta + Kd * self.D_theta
                msg.linear.x = 0.0
            else:
                msg.angular.z = 0.0
                msg.linear.x = Kp * error_dist + Ki * self.I_dist + Kd * self.D_dist
            
            #Changing past parameter
            self.past_dist_error = error_dist
            self.past_theta_error = theta_error
            self.last_time = time.monotonic()

            #Publishing
            self.publisher_.publish(msg)
        else:
            self.isDone = True
            self.publisher_.publish(Twist()) #Stop turtle
            self.get_logger().info("Achived all points!")
        
def main(args=None):
    rclpy.init(args=args)
    node = PID_TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

