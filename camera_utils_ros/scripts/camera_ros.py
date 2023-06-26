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
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, PointCloud2, PointField
from camera_utils_msgs.msg import Frames
from cv_bridge import CvBridge
from std_msgs.msg import Header

import numpy as np
import ros_numpy
import sys

if __name__ == "__main__":

    Resolution = Camera.Resolution

    rospy.init_node("camera_publisher", anonymous=True)

    bridge = CvBridge()

    rgb_topic = rospy.get_param("~rgb_topic", "rgb_image_raw")
    depth_topic = rospy.get_param("~depth_topic", "depth_image_raw")
    camera_info_topic = rospy.get_param("~camera_info_topic", "camera_info")
    camera_pcd_topic = rospy.get_param("~camera_pcd_topic", "camera_pcd")
    frames_topic = rospy.get_param("~frames_topic", "camera_frames")
    

    camera_resolution = rospy.get_param("~camera_resolution", "HD")
    camera_resolution = eval("Resolution." + camera_resolution)
    compressed_image = rospy.get_param("~compressed_image", False)

    fps = rospy.get_param("~fps", 30)
    serial_number = rospy.get_param("~serial_number", "")

    publish_rgb = rospy.get_param("~publish_rgb", None)
    publish_depth = rospy.get_param("~publish_depth", None)
    publish_pcd = rospy.get_param("~publish_pcd", None)
    publish_camera_info = rospy.get_param("~publish_camera_info", None)
    publish_separated_frames = rospy.get_param("~publish_separated_frames", True)

    camera_type = rospy.get_param("~camera_type", None)
    device_idx = rospy.get_param("~device_idx", 0)

    if not camera_type:
        rospy.logerr("No camera type passed")
        exit()

    depth_encoding = None

    if camera_type == "intel":
        camera = IntelRealsense(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
        depth_encoding = "mono16"
    elif camera_type == "zed":
        camera = Zed(camera_resolution=camera_resolution, fps=fps, serial_number=serial_number)
        depth_encoding = "32FC1"
    elif camera_type == "webcam":
        camera = Webcam(device_idx)
    else:
        rospy.logerr("Camera Type " + camera_type + " does not exists!")
        exit()

    rgb_publisher = None
    depth_publisher = None
    camera_info_publisher = None
    pcd_publisher = None
    
    image_type = Image
    pcd_type = PointCloud2

    if compressed_image:
        image_type = CompressedImage
        rgb_topic += "/compressed"

    if publish_depth and publish_rgb:
        if publish_separated_frames:
            depth_publisher = rospy.Publisher(depth_topic, image_type, queue_size=5)
            rgb_publisher = rospy.Publisher(rgb_topic, image_type, queue_size=5)
        else:
            frames_publisher = rospy.Publisher(frames_topic, Frames, queue_size=5)
    elif publish_depth:
        depth_publisher = rospy.Publisher(depth_topic, image_type, queue_size=5)
    elif publish_rgb:
        rgb_publisher = rospy.Publisher(rgb_topic, image_type, queue_size=5)

    if publish_pcd:
        pcd_publisher = rospy.Publisher(camera_pcd_topic, pcd_type, queue_size=5)

    if publish_camera_info and publish_separated_frames:
        camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)

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
    pcd_header.frame_id = "map"

    rgb_cv2_to_imgmsg = bridge.cv2_to_imgmsg
    rgb_encoding = "bgr8"
    if compressed_image:
        rgb_cv2_to_imgmsg = bridge.cv2_to_compressed_imgmsg
        rgb_encoding = "jpg"

    while not rospy.is_shutdown():
        
        t = rospy.Time.now()
                    
        if publish_depth and publish_rgb:
            rgb, depth = camera.get_frames()

            rgb_image = rgb_cv2_to_imgmsg(rgb, rgb_encoding)
            depth_image = bridge.cv2_to_imgmsg(depth, depth_encoding)
            frames = Frames()

            rgb_image.header.stamp = t
            depth_image.header.stamp = t

            if publish_separated_frames:
                rgb_publisher.publish(rgb_image)
                depth_publisher.publish(depth_image)
            else:

                frames.header.stamp = t
                frames.rgb = rgb_image
                frames.depth = depth_image
                frames.camera_info = camera_info

                frames_publisher.publish(frames)
            
        elif publish_rgb:
            rgb = camera.get_rgb()

            rgb_image = rgb_cv2_to_imgmsg(rgb, rgb_encoding)
            rgb_image.header.stamp = t
            rgb_publisher.publish(rgb_image)


        elif publish_depth:
            depth = camera.get_depth()
        
            depth_image = bridge.cv2_to_imgmsg(depth, depth_encoding)
            depth_image.header.stamp = t
            
            depth_publisher.publish(depth_image)

        if publish_pcd:
         
            pc = camera.get_pcd()
          
            pc.transform([[-1, 0,0,0], [0, -1,0,0],[0, 0,-1,0],[0, 0,0,1]])
            pc = np.asarray(pc.points)
           
            pc_array = np.zeros(len(pc), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),])
            
            pc_array['x'] = pc[:, 0]
            pc_array['y'] = pc[:, 1]
            pc_array['z'] = pc[:, 2]
            
            cloud_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=pcd_header.stamp, frame_id=pcd_header.frame_id)
            
            pcd_publisher.publish(cloud_msg)
        
        if publish_camera_info and publish_separated_frames:
            camera_info_publisher.publish(camera_info)
