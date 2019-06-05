#!/usr/bin/env python
"""
OcamNode
for connecting with OCam
Author : C3MX <tutorgaming@gmail.com>

Copyright (c) 2019, Tutorgaming
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import cv2
import liboCams
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class OCamInterface(object):
    """Class For interfacing with oCam Camera
    and Convert to sensor_msgs/Image
    """
    def __init__(self):
        # Parameters
        self.verbose_mode = False
        # OCam
        self.camera_path = None
        self.camera = None
        self.camera_format_list = None
        self.camera_control_list = None
        # Initialize
        self.init_camera(self.verbose_mode)

    def init_camera(self, verbose_flag):
        # Find Camera
        self.camera_path = liboCams.FindCamera('oCam')
        if self.camera_path is None:
            print "Cannot find DevPath : please connect Camera"
            return False

        # Instantiate Camera with Verbose for collecting data
        self.camera = liboCams.oCams(
            self.camera_path,
            verbose=1
        )
        # Collecting Data
        self.gathering_camera_data(self.camera)

        # Camera setting reassign (?)
        for i in range(len(self.camera_format_list)):
            self.camera.Set(self.camera_format_list[i])

        self.camera_name = self.camera.GetName()
        rospy.loginfo("Currently Connecting to Camera : " + str(self.camera_name))

        # Mute Data Printing if verbose is false
        if verbose_flag is False:
            self.camera.Close()
            self.camera = liboCams.oCams(
                self.camera_path,
                verbose=0
            )

        # Start Camera instance !
        self.camera.Start()

    def gathering_camera_data(self, camera):
        """Collecting Camera Data from driver

        Arguments:
            camera {libOCams Camera} -- Camera instance from driver
        """
        self.camera_format_list = camera.GetFormatList()
        self.camera_control_list = camera.GetControlList()

    def get_image(self):
        """Get the image Data and publish into ROS World :)
        """
        # Get Image from Camera
        camera_mat = self.camera.GetFrame()
        rgb = cv2.cvtColor(camera_mat, cv2.COLOR_BayerGB2BGR)
        return rgb


class ImageConverter(object):
    """Class for converting OpenCv's Mat into
    sensor_msgs/Image for ROS's Ecosystem
    """
    def __init__(self):
        self.image_pub = rospy.Publisher(
            "image",
            Image,
            queue_size=10
        )
        self.bridge = CvBridge()

    def publish(self, cv_image):
        """Publish OpenCV Mat in Sensor_msgs/Image Format

        Arguments:
            mat {OpenCV Mat} -- Matrix containing image Data
        """
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    cv_image,
                    "bgr8"
                )
            )
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # Init ROS Node
    rospy.init_node("itutor_ocam")
    ocam = OCamInterface()
    image_converter = ImageConverter()
    # Publish at best effort
    while not rospy.is_shutdown():
        rospy.loginfo("test")
        mat = ocam.get_image()
        image_converter.publish(mat)

