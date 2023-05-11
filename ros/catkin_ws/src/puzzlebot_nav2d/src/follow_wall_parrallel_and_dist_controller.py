#!/usr/bin/env python3

"""
Made by:
    Jose Angel del Angel Dominguez
        joseangeldelangel10@gmail.com
Code description:
Notes:
"""

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowWallController:
    """
    Possible robot states:           
     - turn_on_plain_wall
     - follow_wall     
    """

    def __init__(self, wall_dist=0.3, w_max=np.pi/12, v_max=0.4):
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
        rospy.init_node('follow_wall')

        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.state = "turn_left"                
        self.displaced_angle = None
        self.last_turn_time = None
        self.twist = Twist()
        
        self.front_scan_value = None
        self.front_right_scan_value = None
        self.front_left_scan_value = None
        self.right_scan_value = None
        self.left_scan_value = None

        self.front_scan_angle = 0.0 # TODO let this be defined by the front scan value getter
        self.front_right_scan_angle = None
        self.front_left_scan_angle = None
        self.right_scan_angle = None
        self.left_scan_angle = None

        self.num_sectors = 5.0

        self.scan_fov_deg = 110.0

        self.angle_to_turn_is_computed = False
        self.angle_to_turn = None

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def get_angle_to_turn(self):
        p2_x = self.front_left_scan_value*np.cos(self.front_left_scan_angle)
        p2_y = self.front_left_scan_value*np.sin(self.front_left_scan_angle)
        p1_x = self.front_right_scan_value*np.cos(self.front_right_scan_angle)
        p1_y = self.front_right_scan_value*np.sin(self.front_right_scan_angle)
        if p2_x - p1_x != 0.0:
            angle_to_turn = np.arctan2( (p2_y-p1_y), (p2_x - p1_x))
        else:
            angle_to_turn = 90.0

        self.angle_to_turn = abs(angle_to_turn)
        self.angle_to_turn_is_computed = True
        self.displaced_angle = 0.0
        self.last_turn_time = rospy.get_time()

    def turn_left(self):
        # TODO when we pass this to rover change logic so that the loop is closed using imu data
        if self.scan != None:
            if not self.angle_to_turn_is_computed:
                self.get_angle_to_turn()
                print("angle to turn is: {a}".format(a = np.rad2deg(self.angle_to_turn)))
            else:                                    
                if self.displaced_angle < self.angle_to_turn:                
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.w_max                                
                    self.vel_pub.publish(self.twist)                
                    current_time = rospy.get_time()
                    self.displaced_angle += abs(self.twist.angular.z)*(current_time - self.last_turn_time)
                    self.last_turn_time = current_time
                else:                
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.vel_pub.publish(self.twist)
                    self.state = "follow_wall"

                    self.angle_to_turn = None    
                    self.angle_to_turn_is_computed = False
                    self.displaced_angle = None
                    self.last_turn_time = None


    def get_vector_mag(self, vect):
        return np.sqrt(vect[0]**2 + vect[1]**2)

    def saturate_signal(self, signal, saturation_value):
        if signal > abs(saturation_value):
            result = abs(saturation_value)
        elif signal < -abs(saturation_value):
            result = -abs(saturation_value)
        else:
            result = signal
        return result     

    def follow_wall_controller(self):
        if self.scan != None:
            if self.front_scan_value <= 1.5*self.wall_dist:
                self.state = "turn_left"
            elif self.right_scan_value > 2.5*self.wall_dist:
                self.twist.angular.z = -self.v_max/self.wall_dist
                self.twist.linear.x = self.v_max
                self.vel_pub.publish(self.twist)
            else:
                tangent_vect_p1 = np.array((
                    [[self.right_scan_value*np.cos(self.right_scan_angle)],
                    [self.right_scan_value*np.sin(self.right_scan_angle)]]
                ))
                tangent_vect_p2 = np.array((
                    [[self.front_right_scan_value*np.cos(self.front_right_scan_angle)],
                    [self.front_right_scan_value*np.sin(self.front_right_scan_angle)]]
                ))
                tangent_vect = tangent_vect_p2 - tangent_vect_p1
                print("tangent_vect val is \n {v}".format(v = tangent_vect))
                tangent_vect_unitary = tangent_vect/self.get_vector_mag(tangent_vect)            
                perpendicular_vect_to_robot = tangent_vect_p1 - ( np.dot( tangent_vect_p1.reshape(2,), tangent_vect_unitary.reshape((2,)) ) * tangent_vect_unitary )
                perpendicular_vect_to_robot_unitary = perpendicular_vect_to_robot/self.get_vector_mag(perpendicular_vect_to_robot)
                wall_dist_error_vect = perpendicular_vect_to_robot - self.wall_dist*perpendicular_vect_to_robot_unitary

                parallel_kp = 0.5
                wall_dist_kp = 0.5

                follow_wall_vect = wall_dist_kp*wall_dist_error_vect + parallel_kp*tangent_vect 

                self.twist.angular.z = np.arctan2( follow_wall_vect[1], follow_wall_vect[0] )            
                self.twist.linear.x = self.v_max

                self.vel_pub.publish(self.twist)

    def get_laser_index_from_angle(self, angle_in_rad):
        scan_fov_rad = np.deg2rad(self.scan_fov_deg)        
        angle_index = ( (angle_in_rad + (scan_fov_rad/2.0) ) * (len(self.scan.ranges) - 1.0) )/(scan_fov_rad)
        angle_index = int( round(angle_index) )
        return angle_index

    def get_ray_sectors_info(self):
        if self.scan is not None:
            """
            left = self.scan.ranges[-1]
            half = int(len(self.scan.ranges) // 2)
            front = self.scan.ranges[half]
            right = self.scan.ranges[0]
            return (left, front, right)
            """
            #print("lidar angle min max {mi}, {ma}".format(mi = self.scan.angle_min, ma = self.scan.angle_max))
            sector_size = (self.scan.angle_max - self.scan.angle_min)/float(self.num_sectors)
            
            self.right_scan_angle = self.scan.angle_min + (1.0/2.0)*sector_size
            self.front_right_scan_angle = self.scan.angle_min + (3.0/2.0)*sector_size
            self.front_scan_angle = self.scan.angle_min + (5.0/2.0)*sector_size
            self.front_left_scan_angle = self.scan.angle_min + (7.0/2.0)*sector_size
            self.left_scan_angle = self.scan.angle_min + (9.0/2.0)*sector_size

            right_scan_values = self.scan.ranges[
                self.get_laser_index_from_angle(self.scan.angle_min) :
                self.get_laser_index_from_angle(self.scan.angle_min + 1.0*sector_size)
                ]
            front_right_scan_values = self.scan.ranges[
                self.get_laser_index_from_angle(self.scan.angle_min + 1.0*sector_size) :
                self.get_laser_index_from_angle(self.scan.angle_min + 2.0*sector_size)
                ]
            front_scan_values = self.scan.ranges[
                self.get_laser_index_from_angle(self.scan.angle_min + 2.0*sector_size) :
                self.get_laser_index_from_angle(self.scan.angle_min + 3.0*sector_size)
                ]
            front_left_scan_values = self.scan.ranges[
                self.get_laser_index_from_angle(self.scan.angle_min + 3.0*sector_size) :
                self.get_laser_index_from_angle(self.scan.angle_min + 4.0*sector_size)
                ]
            left_scan_values = self.scan.ranges[
                self.get_laser_index_from_angle(self.scan.angle_min + 4.0*sector_size) :
                self.get_laser_index_from_angle(self.scan.angle_max)
                ]
            
            self.right_scan_value = min(right_scan_values)
            self.front_right_scan_value = min(front_right_scan_values)
            self.front_scan_value = min(front_scan_values)
            self.front_left_scan_value = min(front_left_scan_values)
            self.left_scan_value = min(left_scan_values)
        
    def main(self):
        if self.scan is not None:
            self.get_ray_sectors_info()        
            if self.state == "turn_left":            
                self.turn_left()
            elif self.state == "follow_wall":            
                self.follow_wall_controller()
        self.rate.sleep()        

if __name__ == '__main__':    
    rhw = FollowWallController()
    while not rospy.is_shutdown():                
            rhw.main()