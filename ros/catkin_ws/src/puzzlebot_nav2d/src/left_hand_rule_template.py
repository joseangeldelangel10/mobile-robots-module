#!/usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class RightHandRuleController:
    """
    Possible robot states:      
     - go_straight 
     - turn_on_plain_wall
     - turn_inside_corner
     - turn_outside_corner
    """

    def __init__(self, wall_dist=0.6, w_max=-np.pi/12, v_max=0.4):
        """
        Arguments
        --------------
        wall_dist  : float
           Desired distance to wall
        w_max  : float
           Max angular velocity
        v_max  : float
           Max linear velocity
        """
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.state = "go_straight"
        self.wall_is_found = False
        self.block_state_change = False
        self.displaced_angle = None
        self.last_turn_time = None

    def reset_turn_values(self):
        self.displaced_angle = 0.0
        self.last_turn_time = rospy.get_time()        

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def create_vel_msg(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x
        return msg
    
    def go_straight(self):
        if self.scan != None:
            linear_x = self.v_max
            angular_z = 0.0
            msg = self.create_vel_msg(linear_x, angular_z)
            self.vel_pub.publish(msg)

    def icr_msg(self,r,w):
        if self.scan != None:
            linear_x = w*r
        return linear_x

    def turn_right(self):
        if self.scan != None:            
            if self.displaced_angle < np.pi/2 + 0.3:                
                linear_x = 0.0
                angular_z = self.w_max            
                msg = self.create_vel_msg(linear_x, angular_z)
                self.vel_pub.publish(msg)                
                current_time = rospy.get_time()
                self.displaced_angle += abs(angular_z)*(current_time - self.last_turn_time)
                self.last_turn_time = current_time
            else:                
                self.block_state_change = False
                msg = self.create_vel_msg(0.0, 0.0)
                self.vel_pub.publish(msg)

        
    def right_hand_controller(self):
        if self.scan != None:
            # TODO complete linear_x and angular_z vals
            dist_at_180 = self.scan.ranges[-1]
            dist_at_135 = self.scan.ranges[3*len(self.scan.ranges) // 4]
            l3 = np.sqrt( (dist_at_180**2) + (dist_at_135**2) - 2*dist_at_180*dist_at_135*(np.sqrt(2)/2) )
            y1 = np.arcsin((dist_at_135*(np.sqrt(2)/2))/l3)
            ang_err = y1 - np.pi/2
            kp = -0.8
            kp2 = 5.5
            angular_z = kp*ang_err
            
            relative_euclidean_distance_to_wall = dist_at_180 - self.wall_dist
            if relative_euclidean_distance_to_wall <= (self.wall_dist/2):
                angular_z += kp2*relative_euclidean_distance_to_wall
            
            if abs(angular_z) < 0.8:
                linear_x = self.v_max                
            else:
                linear_x = 0.0

            if dist_at_180 >= 1.2:
                angular_z = 1.0 #turn_left 

            msg = self.create_vel_msg(linear_x, angular_z)
            self.vel_pub.publish(msg)

    def get_left_front_right_scans(self):
        if self.scan is not None:
            left = self.scan.ranges[-1]
            half = int(len(self.scan.ranges) // 2)
            front = self.scan.ranges[half]
            right = self.scan.ranges[0]
            return (left, front, right)
        else:
            return (None, None, None)
        
    def main(self):
        if self.state == "go_straight":
            self.go_straight()
        elif self.state == "turn_right":            
            self.turn_right()
        elif self.state == "right_hand_controller":            
            self.right_hand_controller()        


def generate_test_scan(straight_wall=False):
    """Function used for testing. Will create and return a LaserScan message"""
    scan = LaserScan()
    scan.angle_min = -np.pi / 2
    scan.angle_max = np.pi / 2
    num_scans = 720  # 4 lines per degree
    scan.angle_increment = np.pi / num_scans
    scan.range_min = 0.1
    scan.range_max = 30

    scan.ranges = np.arange(0, 720.0) / 10.0
    if straight_wall:  # The wall is on the right at a distance of 10m
        scan.ranges[:400] = scan.range_max
        dth = np.arange(0, 320) * scan.angle_increment
        for i in range(320):
            scan.ranges[719 - i] = 10 / np.cos(dth[i])

    return scan


if __name__ == '__main__':

    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('follow_right_hand_wall')
    rhw = RightHandRuleController()
    while not rospy.is_shutdown():        
        left_scan, front_scan, right_scan = rhw.get_left_front_right_scans()
        if left_scan is not None and front_scan is not None and right_scan is not None:
            if not rhw.wall_is_found and front_scan > rhw.wall_dist:
               rhw.state == "go_straight"
            elif not rhw.wall_is_found and front_scan <= rhw.wall_dist: 
                rhw.wall_is_found = True
            elif rhw.wall_is_found and not rhw.block_state_change:                
                previous_state = rhw.state
                if front_scan <= rhw.wall_dist:
                    rhw.block_state_change = True
                    rhw.reset_turn_values()
                    rhw.state = "turn_right"                    
                else: 
                    rhw.state = "right_hand_controller"
                if previous_state != rhw.state:
                    print(rhw.state)

            rhw.main()