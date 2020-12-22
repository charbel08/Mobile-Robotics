#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys
from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import DynamicReconConfig

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        #self.control = ???
        
        self.current_error = current_error
        self.curr_error_deriv = (self.current_error - self.prev_error)/self.dt
        self.control = self.Kp*(self.current_error + self.Td * self.curr_error_deriv)
        self.sum_error += current_error
        if self.Ti != 0:
            integrator = self.Kp * (1/self.Ti * self.sum_error)
            # Making sure integral is within acceptable bounds to avoid integrator windup
            if integrator <= 0.5 and integrator >= -0.5:
                self.control += integrator
        self.prev_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        
    def get_control(self):
        return self.control
        
    def update_gains(self, Kp, Td, Ti):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50
        self.pid = PID(0.33, 3.3, 12, float(1)/float(self.hz))

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        # self.cmd_pub = ??
        self.cmd_pub = rospy.Publisher('/husky_1/cmd_vel', Twist, queue_size=10)

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        # self.laser_sub = ???
        self.laser_sub = rospy.Subscriber('/husky_1/scan', LaserScan, self.laser_scan_callback)
        self.cte_pub = rospy.Publisher('/husky_1/cte', Float32, queue_size=10)
        self.srv = Server(DynamicReconConfig, self.dynamic_recon_callback)
    
    def dynamic_recon_callback(self, config, level):
        # Dynamically updates the gains in PID when the server is called
        self.pid.update_gains(config.Kp, config.Td, config.Ti)
        return config
        
        
    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control.
        
        # cmd.angular.z = ???
        
        # Taking the closest point to the left of the robot
        distance_from_wall = min(msg.ranges[:340])
        # Computing cross track error
        cte = distance_from_wall - self.desired_distance_from_wall
        # Publishing to /husky_1/cte
        self.cte_pub.publish(cte)
        # Creating Twist data to command the husky
        cmd = Twist()
        # Giving the robot its desired speed and setting all other parameters to 0
        cmd.linear.x = self.forward_speed
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        # Updating the PID
        self.pid.update_control(cte)
        # Getting the command for angluar velocity
        cmd.angular.z = self.pid.get_control()
        # Publishing new command to /husky_1/cmd_vel
        self.cmd_pub.publish(cmd)
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()

