#!/usr/bin/env python
#publishes center of clusters in a PointCloud2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
import numpy as np
import struct
from sklearn.cluster import DBSCAN
import cv2
from cv_bridge import CvBridge

class center_of_clusters(Node):
    def __init__(self):
        super().__init__("detect_radius")
        
        self.get_logger().info("this is working")
        #self.pcd_publisher = self.create_publisher(PointCloud2,'center_of_clusters',10)
        self.subscription = self.create_subscription(PointCloud2,
            "/velodyne_points",self.listener_callback,10)
        self.subscription = self.create_subscription(PointCloud2,"/depth/image_rect_raw",self.depth_callback,10)
        self.subscription = self.create_subscription(Image, "/color/image_raw", self.image_callback,10)

        #self.allPointsPublisher = self.create_publisher(PointCloud2,'all_points',10)
        timer_period = .5
        #self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pcd = PointCloud2()
        self.img = Image()
        self.depth_img = PointCloud2()
    #def timer_callback(self):
        #self.pcd_publisher.publish(self.pcd)
    
    def listener_callback(self,msg:PointCloud2):
        #print("data received")
        self.pcd = point_cloud(msg,self.img,self.depth_img,self)
    #def mypublishAllPoints(self,data:PointCloud2):
        #self.allPointsPublisher.publish(data)
    def image_callback(self,msg):
        self.img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("img", self.img)
        cv2.waitKey(1)

        self.pcd = point_cloud(msg,self.img,self.depth_img,self)
    def depth_callback(self,msg:PointCloud2):
        self.depth_img = msg

def point_cloud(msg,img,depth_img,node):
    points = np.array(list(read_points(msg)))
    points = points[:,0:3]
    infMask = ~np.isnan(points).any(axis=1)
    points = points[infMask, :]

    mask = np.empty(len(points),dtype=bool)
    threshold = 1
    for i, point in enumerate(points):
        x = point[0]
        y = point[1]
        z = point[2]
        #print("point: "+str(point))
        if math.sqrt(x*x+y*y+z*z) > threshold:
            mask[i] = True
        
    points = points[~mask]
    #print(points)

    #angle
    angles = np.empty(len(points))
    for i, point in enumerate(points):
        x = point[0]
        y = point[1]
        z = point[2]
        angle = math.degrees(math.acos(x/math.sqrt(x*x+y*y)))
        angles[i] = angle
        #print("angle: "+str(angle))
    #print(angles)

    coordinates = np.empty((len(points),2))
    lat = 30.00
    long = 30.000
    for i, point in enumerate(points):
        x = point[0]
        y = point[1]
        z = point[2]
        distance = math.sqrt(x*x+y*y+z*z)

        easting = distance*math.cos(math.radians(angles[i]))
        northing = distance*math.sin(math.radians(angles[i]))

        rad_earth = 6378.137
        pi = math.pi

        change_lat = (northing/ ((2*pi/360)*rad_earth))/1000
        change_long = (easting/ ((2*pi/360)*rad_earth))/1000

        coordinates[i][0] = lat+change_lat
        coordinates[i][1] = long+change_long

    #print(coordinates)
    print("num of points within "+str(threshold)+" meter(s): "+str(len(points)))
    for (point, angle, coordinate) in zip(points,angles,coordinates):
        print("point: "+str(point)+" angle: "+str(angle)+" coordinates: "+str(coordinate))

import sys
from collections import namedtuple
import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def main(args=None):
    rclpy.init(args=args)
    publisher = center_of_clusters()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()
