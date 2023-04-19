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
from ros_numpy import numpify

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
        self.rate = rospy.Rate(5.0)
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
        self.map2d = np.zeros((map_width, map_height), dtype=np.int8) # For computation
        
        self.num_elems_in_laser_scan = None
        self.scan = None
        self.odom = None


    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def odom_callback(self, msg):
        """Called when a new odometry message is available. """
        self.odom = msg

    def base_cordinates_to_origin_cordinates(self, point_in_base_cords):
        """
        point_in_base_cords -> np.array of shape = (3, 1)
        """
        x_odom = self.odom.pose.pose.position.x
        y_odom = self.odom.pose.pose.position.y
        z_odom = self.odom.pose.pose.position.z
        th_odom = 2*np.arcos(self.odom.pose.pose.orientation.z)

        point_in_origin_cords = np.array([[np.cos(th_odom),-np.sin(th_odom),0,x_odom],[np.sin(th_odom),np.cos(th_odom),0,y_odom],[0,0,1,z_odom],[0,0,0,1]])
        return point_in_origin_cords

    def origin_cordinates_to_map_coordinates(self, point_in_origin_cords):
        """
        point_in_origin_cords -> np.array of shape = (3, 1)
        """
        point_in_map_cords = np.array([[1,0,0,],[0,-1,0,],[0,0,-1],[0,0,0,1]])
        return point_in_map_cords

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

        #------------------------------------------------------
        # Your code here
        # Convert the polar coordinate to a cartesian coordinate
        x = r*np.cos(th)
        y = r*np.sin(th)
        #-----------------------------------------------------

        return (x, y)
    
    def point_is_inside_pixel(self, point_x, point_y, pixel_i, pixel_j):
        """
        point_x -> x cord on map frame
        point_y -> y cord on map frame        
        """
        pass 

    def get_ith_laser_info(self, i):
        """
        function that returns the angle and len of the ith laser scan
        """
        return (ray_len, ray_ang)

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

        pass

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

                orig_m, xy_m = scan_to_map_coordinates(self.scan, self.odom, self.map.info.origin)

                print(orig_m)
                for xy_ in xy_m:
                    #print(xy_)
                    ray_to_pixels(orig_m[0], orig_m[1], xy_[0], xy_[1], self.map.info.resolution, self.map2d)


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
    width = rospy.get_param("/mapper/width", 400)
    height = rospy.get_param("/mapper/height", 400)
    resolution = rospy.get_param("/mapper/resolution", 0.1) # meters per pixel

    Mapper(width, height, resolution).mapit()



