#!/usr/bin/env python3
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
# -*- coding: utf-8 -*-
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class Converter(Node):

    def __init__(self):
        super().__init__('converter')

        self._frame_id = self._get_frame_id('frame_id')
        self._child_frame_id = self._get_frame_id('child_frame_id')

        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(Odometry, '~/input_odom', self._callback, 1)

    def _get_frame_id(self, name):
        self.declare_parameter(name, "")
        return self.get_parameter(name).get_parameter_value().string_value

    def _callback(self, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp

        if self._frame_id:
            tf.header.frame_id = self._frame_id
        else:
            tf.header.frame_id = msg.header.frame_id

        if self._child_frame_id:
            tf.child_frame_id = self._child_frame_id
        else:
            tf.child_frame_id = msg.child_frame_id

        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.pose.pose.orientation

        self._broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    relay = Converter()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
