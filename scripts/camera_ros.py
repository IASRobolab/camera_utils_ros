#!/usr/bin/env python3

'''----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022
#
# author: Federico Rollo
# mail: rollo.f96@gmail.com
#
# Institute: Leonardo Labs (Leonardo S.p.a - Istituto Italiano di tecnologia)
#
# This file is part of camera_utils_ros. <https://github.com/IASRobolab/camera_utils_ros>
#
# camera_utils_ros is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# camera_utils_ros is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------'''

from camera_utils.cameras.IntelRealsense import IntelRealsense
from camera_utils.cameras.Zed import Zed
from camera_utils.cameras.Webcam import Webcam
from camera_utils.cameras.CameraInterface import Camera

import rospy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import sys
import struct
import select

def stop_loop(stop_entry):
    '''
    Used to quit an infinite loop with a char/string entry
    '''
    rlist = select.select([sys.stdin], [], [], 0.001)[0]
    if rlist and sys.stdin.readline().find(stop_entry) != -1:
        return True
    return False


if __name__ == "__main__":

    Resolution = Camera.Resolution

    rospy.init_node("camera_publisher", anonymous=True)

    bridge = CvBridge()

    rgb_topic = rospy.get_param("~rgb_topic", "rgb_image_raw")
    depth_topic = rospy.get_param("~depth_topic", "depth_image_raw")
    camera_info_topic = rospy.get_param("~camera_info_topic", "camera_info")
    camera_pcd_topic = rospy.get_param("~camera_pcd_topic", "camera_pcd")

    camera_resolution = rospy.get_param("~camera_resolution", "HD")
    camera_resolution = eval("Resolution." + camera_resolution)
    fps = rospy.get_param("~fps", 30)
    serial_number = rospy.get_param("~serial_number", "")
    pcd_header = rospy.get_param("~pcd_header", "map")

    publish_rgb = rospy.get_param("~publish_rgb", None)
    publish_depth = rospy.get_param("~publish_depth", None)
    publish_camera_info = rospy.get_param("~publish_camera_info", None)
    publish_pcd = rospy.get_param("~publish_pcd", None)

    camera_type = rospy.get_param("~camera_type", None)

    if not camera_type:
        rospy.logerr("No camera type passed")
        exit()

    if camera_type == "intel":
        camera = IntelRealsense(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
    elif camera_type == "zed":
        camera = Zed(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
    elif camera_type == "webcam":
        camera = Webcam()
    else:
        rospy.logerr("Camera Type " + camera_type + " does not exists!")
        exit()

    rgb_publisher = rospy.Publisher(rgb_topic, Image, queue_size=5)
    depth_publisher = rospy.Publisher(depth_topic, Image, queue_size=5)
    camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)
    pointcloud_publisher = rospy.Publisher(camera_pcd_topic, PointCloud2, queue_size=3)

    intr = None
    camera_info = CameraInfo()

    try: 
        intr =  camera.get_intrinsics()
        camera_info.K[0] = intr['fx']
        camera_info.K[4] = intr['fy']
        camera_info.K[2] = intr['px']
        camera_info.K[5] = intr['py']
        camera_info.width = intr['width']
        camera_info.height = intr['height']
    except KeyError:
        pass

    image_dim = camera.get_rgb().shape
    pcd_header = Header()
    pcd_header.frame_id = pcd_header

    while not stop_loop(ord('q')):


        if publish_pcd:
            rgb, depth = camera.get_frames()

            points = []

            for u in range(image_dim[0]):
                for v in range(image_dim[1]):

                    z = depth[u, v] * 0.001
                    x = ((u - intr["px"]) * z) / intr["fx"]
                    y = ((v - intr["py"]) * z) / intr["fy"]

                    color = rgb[u, v]

                    r = int(color[0])
                    g = int(color[1])
                    b = int(color[2])
                    a = 255

                    color = struct.unpack('I', struct.pack('BBBB', r, g, b, a))[0]

                    points.append([x, y, z, color])
            
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1)]


            pcd = point_cloud2.create_cloud(pcd_header, fields, points)
            pcd.header.stamp = rospy.Time.now()
            pointcloud_publisher.publish(pcd)
            rgb_publisher.publish(bridge.cv2_to_imgmsg(rgb, "bgr8"))
            depth_publisher.publish(bridge.cv2_to_imgmsg(depth, "mono16"))

        elif publish_depth and publish_rgb:
            rgb, depth = camera.get_frames()
            rgb_publisher.publish(bridge.cv2_to_imgmsg(rgb, "bgr8"))
            depth_publisher.publish(bridge.cv2_to_imgmsg(depth, "mono16"))

        elif publish_rgb:
            rgb = camera.get_rgb()
            rgb_publisher.publish(bridge.cv2_to_imgmsg(rgb, "bgr8"))

        elif publish_depth:
            depth = camera.get_depth()
            depth_publisher.publish(bridge.cv2_to_imgmsg(depth, "mono16"))

        
        if publish_camera_info:
            camera_info_publisher.publish(camera_info)
