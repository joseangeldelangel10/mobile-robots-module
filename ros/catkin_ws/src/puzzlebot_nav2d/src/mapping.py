#!/usr/bin/env python  
""" Node that generates a map of the environment based on the laser scan data.
and the odometry data.

Author: kjartan@tec.mx (Kjartan Halvorsen) with help from github copilot

Notes.
1) The scan data give information about free space as well as obstacles. Each ray in the scan will cover a number
of pixels in the map. The map should be updated by setting the pixels covered by the ray to 0 (free) and the last pixel
to occupied (100). The map should be updated only if the ray range is less than the max_range of the scan.
2) You should determine the number of points in each scan ray by multiplying the range of the ray by the map resolution.
Then you convert these points (each corresponding to a pixel) from a robot frame to a map frame using the odometry data.
3) The map should be updated only if the robot has moved a certain distance since the last update. This is to
avoid updating the map too often, since it is a somewhat expensive operation.
4) It can be more efficient to use numpy arrays for the rigid transformations needed to convert scans
to map coordinates. To make this work, you need to convert the geometry_msgs/TransformStamped to a numpy array.
See https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
With a transform matrix T, you can transform a number of points at once by collecting the points in a numpy array
and multiplying the array with T.
To use numpify, you need to install the package ros-meldic-ros-numpy.


"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

"""
joses notes:
- in puzz sim lidar covers 180 deg
- lidar vals go from 0 to 180 deg
"""

class Mapper:
    def __init__(self, map_width, map_height, map_resolution):
        """
        Arguments
        ---------
        map_width : float
            Width of map in pixels (x-axis)
        map_height : float
            Height of map in pixels (y-axis)
        map_resolution : float
            Resolution of map in meter per pixel
        """
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.odom_listener = rospy.Subscriber('/true_odometry', Odometry,
                                              self.odom_callback)
        self.map_pub = rospy.Publisher('/map' , OccupancyGrid, queue_size=1 )
        self.rate = rospy.Rate(2)
        self.map = OccupancyGrid()
        self.map.info.map_load_time = rospy.Time.now()
        self.map.info.resolution = map_resolution
        self.map.info.width = map_width
        self.map.info.height = map_height
        self.map.info.origin.position.x = -(map_width*map_resolution)/2.0
        self.map.info.origin.position.y = (map_height*map_resolution)/2.0
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 1.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.0

        self.map.data = np.zeros(map_width*map_height, dtype=np.int8)
        self.map.data[:] = -1 # Unknown
        self.map2d = np.ones((map_height, map_width), dtype=np.int8)*-1 # For computation
        
        self.num_elems_in_laser_scan = None
        self.scan = None
        self.odom = None    
        
        self.block_new_laser_scan_and_odom_data = False


    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        if not self.block_new_laser_scan_and_odom_data:
            self.scan = msg
            self.num_elems_in_laser_scan = len(self.scan.ranges)                        

    def odom_callback(self, msg):
        """Called when a new odometry message is available. """
        if not self.block_new_laser_scan_and_odom_data:
            self.odom = msg

    def base_cordinates_to_origin_cordinates(self, point_in_base_cords):
        """
        Arguments
        ---------
        point_in_base_cords -> np.array of shape = (3, 1)

        Returns
        ---------
        point_in_origin_cords -> np.array of shape = (3, 1)
        """
        x_odom = self.odom.pose.pose.position.x
        y_odom = self.odom.pose.pose.position.y
        z_odom = self.odom.pose.pose.position.z
        #th_odom = 2*np.arccos(self.odom.pose.pose.orientation.z)
        _, _, th_odom = euler_from_quaternion([self.odom.pose.pose.orientation.x, 
                                             self.odom.pose.pose.orientation.y, 
                                             self.odom.pose.pose.orientation.z,
                                             self.odom.pose.pose.orientation.w])

        transofrm = np.array([[np.cos(th_odom),-np.sin(th_odom),0.0,x_odom],
                            [np.sin(th_odom),np.cos(th_odom),0.0,y_odom],
                            [0.0, 0.0, 1.0, z_odom],
                            [0.0, 0.0, 0.0, 1.0]])
        point_in_base_cords_formated = np.append(point_in_base_cords, 1.0)
        point_in_base_cords_formated = point_in_base_cords_formated.reshape(4,1)
        point_in_origin_cords = np.matmul(transofrm,point_in_base_cords_formated)
        
        return point_in_origin_cords[:][:3]

    def origin_cordinates_to_map_coordinates(self, point_in_origin_cords):
        """
        Arguments
        ---------
        point_in_origin_cords -> np.array of shape = (3, 1)
        
        Returns
        ---------
        point_in_map_cords -> np.array of shape = (3, 1)
        """
        transform = np.array([[1.0, 0.0, 0.0,(self.map.info.width * self.map.info.resolution )/2.0],
                              [0.0, -1,0, (self.map.info.height * self.map.info.resolution )/2.0],
                              [0.0, 0.0, -1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
        point_in_origin_cords_formated = np.append(point_in_origin_cords, 1)
        point_in_origin_cords_formated = point_in_origin_cords_formated.reshape((4,1))
        point_in_map_cords = np.matmul(transform,point_in_origin_cords_formated)

        return point_in_map_cords[:][:3]

    def polar_to_cartesian(self, r, th):
        """ Convert a polar coordinate to a cartesian coordinate.
        Arguments
        ---------
        r : float
            The radius
        th : float
            The angle
        Returns
        -------
        (x, y) : tuple of floats
            The cartesian coordinates
        
        """

        x = r*np.cos(th)
        y = r*np.sin(th)

        return (x, y)
    
    def point_is_inside_pixel(self, point_x, point_y, pixel_i, pixel_j):
        """
        point_x -> x cord on map frame
        point_y -> y cord on map frame        
        """
        if pixel_i >= self.map.info.height or pixel_j >= self.map.info.width:
            return False
        
        pixel_x_min, pixel_x_max = (pixel_j*self.map.info.resolution, pixel_j*self.map.info.resolution + self.map.info.resolution)
        pixel_y_min, pixel_y_max = (pixel_i*self.map.info.resolution, pixel_i*self.map.info.resolution + self.map.info.resolution)
        if (point_x >= pixel_x_min and point_x <= pixel_x_max) and (point_y >= pixel_y_min and point_y <= pixel_y_max):
            return True
        else:
            return False

    def get_pixel_center(self,pixel_i, pixel_j):

        pixel_x_min, pixel_x_max = (pixel_j*self.map.info.resolution, pixel_j*self.map.info.resolution + self.map.info.resolution)
        pixel_y_min, pixel_y_max = (pixel_i*self.map.info.resolution, pixel_i*self.map.info.resolution + self.map.info.resolution)

        center_j = pixel_x_min + (pixel_x_max - pixel_x_min)/2
        center_i = pixel_y_min + (pixel_y_max - pixel_y_min)/2

        return center_j,center_i


    def get_ith_laser_info(self, i):
        """
        function that returns the angle and len of the ith laser scan
        """
        if self.num_elems_in_laser_scan is not None:
            ray_len = self.scan.ranges[i]
            ray_ang = self.scan.angle_min + self.scan.angle_increment*i
        return (ray_len, ray_ang)
    
    def set_new_pixel_val(self, pixel_i, pixel_j, new_val):        
        current_pixel_val = self.map2d[pixel_i, pixel_j]
        if current_pixel_val == -1:
            self.map2d[pixel_i, pixel_j] = new_val            

    def find_pixel_that_contains_point(self, point_x, point_y):
        """ function that recieves the cordinates x, y of a point in the map frame and 
        returns the indexes of the pixel that cointains such point to it"""
        pixel_i = point_y//self.map.info.resolution
        pixel_j = point_x//self.map.info.resolution
        if pixel_i >= 0.0 and pixel_i < self.map.info.height and pixel_j >= 0.0 and pixel_j < self.map.info.width:
            return (int(pixel_i), int(pixel_j))
        else:
            return (None, None) # meaning invalid pixel val 

    def ray_to_pixels(self, xr, yr, x, y):
        """ Set the pixels along the ray with origin (xr,yr) and with range ending at (x,y) to 0 (free) and the end point to 100 (occupied).
        Arguments
        ---------
        xr : float
            x-coordinate of the robot in the map frame
        yr : float
            y-coordinate of the robot in the map frame
        x : ndarray
            x coordinate of the point where laser collides
        y : ndarray
            x coordinate of the point where laser collides
        """

        m1 = (y - yr)/(x - xr)
        b1 = -m1*xr + yr

        i_Puzzlebot, j_Puzzlebot = self.find_pixel_that_contains_point( xr, yr)        
        i_Punto, j_Punto = self.find_pixel_that_contains_point( x, y)        
        self.set_new_pixel_val(i_Puzzlebot,j_Puzzlebot,0.0)
        self.set_new_pixel_val(i_Punto,j_Punto,100.0)

        min_i = min(i_Puzzlebot,i_Punto)
        max_i = max(i_Puzzlebot,i_Punto)

        min_j = min(j_Puzzlebot,j_Punto)
        max_j = max(j_Puzzlebot,j_Punto)

        for i in range(min_i,max_i+1):
            for j in range(min_j,max_j+1):
                xp, yp =  self.get_pixel_center(i, j)

                m2 = -1/m1
                b2 = -m2*xp + yp

                x_var = (b1 - b2)/(m2 - m1)
                y_var =  m1*x_var + b1

                if self.point_is_inside_pixel(x_var,y_var,i,j):
                    self.set_new_pixel_val(i,j,0)


    def mapit(self):
        while not rospy.is_shutdown():

            if self.scan is not None and self.odom is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) For each ray in the scan, calculate the corresponding
                #    position in the map by transforming the ray from the robot
                #    frame to the map frame, using the odometry data. 
                #    It is convenient to define the map frame as having its origin
                #    in the pixel (0,0), and directions corresponding to the 
                #    rows and pixels of the map (occupancy grid).
                #    is defined as the frame of the first laser scan, when the robot
                #    is initialized.
                # 2) If the ray range is less than max_range, then set the map pixel
                #    corresponding to the end point of the ray to 100 (occupied).
                # 3) Set pixels along the ray to 0 (free).
                #--------------------------------------------------------------
                #--------------------------------------------------------------                
                self.block_new_laser_scan_and_odom_data = True
                print(" ========= num elems in laser scan: {n}".format(n = self.num_elems_in_laser_scan))        
                for s in range(self.num_elems_in_laser_scan):                
                    scan_len, scan_ang = self.get_ith_laser_info(s)
                    print("scan ang: {m}, scan len: {n}".format(m=scan_ang, n = scan_len))        
                    scan_cartesian_cords_in_b = self.polar_to_cartesian(scan_len, scan_ang)
                    scan_cartesian_cords_in_b_formated_1 = np.array([scan_cartesian_cords_in_b[0], scan_cartesian_cords_in_b[1], 0.0])
                    scan_cartesian_cords_in_b_formated_2 = scan_cartesian_cords_in_b_formated_1.reshape((3,1))
                    scan_cartesian_cords_in_map = self.origin_cordinates_to_map_coordinates( self.base_cordinates_to_origin_cordinates(scan_cartesian_cords_in_b_formated_2) )
                    print("scan_cartesian_cords_in_map: {c}".format(c = scan_cartesian_cords_in_map))
                    scan_collision_px_i, scan_collision_px_j = self.find_pixel_that_contains_point(scan_cartesian_cords_in_map[0][0], scan_cartesian_cords_in_map[1][0])
                    print("colliding pixel cords: {i}, {j}".format(i=scan_collision_px_i, j = scan_collision_px_j))
                    if scan_collision_px_i is not None and scan_collision_px_j is not None:
                        puzzlebot_position_in_b = np.zeros((3,1))
                        puzzlebot_position_in_map = self.origin_cordinates_to_map_coordinates( self.base_cordinates_to_origin_cordinates(puzzlebot_position_in_b) )    
                        print("puzzlebot possition in_map: {c}".format(c = puzzlebot_position_in_map))
                        self.ray_to_pixels(puzzlebot_position_in_map[0][0], puzzlebot_position_in_map[1][0], scan_cartesian_cords_in_map[0][0], scan_cartesian_cords_in_map[1][0])                  
                self.block_new_laser_scan_and_odom_data = False  
                # Publish the map
                np.copyto(self.map.data,  self.map2d.reshape(-1)) # Copy from map2d to 1d data, fastest way
                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)
            self.rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Mapper')
    width = rospy.get_param("/mapper/width", 300)
    height = rospy.get_param("/mapper/height", 300)
    resolution = rospy.get_param("/mapper/resolution", 0.1) # meters per pixel

    Mapper(width, height, resolution).mapit()



