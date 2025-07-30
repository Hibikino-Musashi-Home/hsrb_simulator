#!/usr/bin/env python3
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
from compressed_rgbd_msgs.msg import CompressedRGBD

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
)


class CompressedRgbdPublisher(Node):

    def __init__(self):
        super().__init__("cmpressed_rgbd_publisher")

        sub_rgb_info = message_filters.Subscriber(
            self, CameraInfo, "rgb/camera_info", qos_profile=qos_profile_sensor_data)
        sub_rgb_img = message_filters.Subscriber(
            self, CompressedImage, "rgb/image", qos_profile=qos_profile_sensor_data)
        sub_depht_info = message_filters.Subscriber(
            self, CameraInfo, "depth/camera_info", qos_profile=qos_profile_sensor_data)
        sub_depth_img = message_filters.Subscriber(
            self, CompressedImage, "depth/image", qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb_info, sub_rgb_img, sub_depht_info, sub_depth_img], 10, 0.1)
        self.ts.registerCallback(self._callback)

        self._rgbd_pub = self.create_publisher(CompressedRGBD, "~/image", qos_profile_sensor_data)

    def _callback(self, rgb_info, rgb_img, depth_info, depth_msg):
        msg = CompressedRGBD()
        msg.header = rgb_info.header
        msg.rgb_camera_info = rgb_info
        msg.rgb = rgb_img
        msg.depth_camera_info = depth_info
        msg.depth = depth_msg
        self._rgbd_pub.publish(msg)


def main():
    rclpy.init()
    pub = CompressedRgbdPublisher()
    rclpy.spin(pub)
    pub.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
