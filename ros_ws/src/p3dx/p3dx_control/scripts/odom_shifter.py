#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OdomShifter:
    def __init__(self):
        rospy.init_node('odom_shifter_node')

        self.init_x = self._get_spawn_param('initial_pose_x')
        self.init_y = self._get_spawn_param('initial_pose_y')
        self.init_a = self._get_spawn_param('initial_pose_a')

        self.sub = rospy.Subscriber('/RosAria/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/RosAria/odom_', Odometry, queue_size=10)

        rospy.loginfo(
            'Nó Odom Shifter iniciado. Lendo /RosAria/odom e publicando /RosAria/odom_ com offset: X=%.2f, Y=%.2f, Yaw=%.2f',
            self.init_x,
            self.init_y,
            self.init_a,
        )

    def _get_spawn_param(self, name):
        private_name = '~' + name
        global_name = '/' + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        if rospy.has_param(global_name):
            return rospy.get_param(global_name)
        return 0.0

    def odom_callback(self, msg):
        shifted_odom = Odometry()
        shifted_odom.header = msg.header
        shifted_odom.child_frame_id = msg.child_frame_id

        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        rot = msg.pose.pose.orientation
        _, _, theta_odom = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

        new_x = self.init_x + (x_odom * math.cos(self.init_a)) - (y_odom * math.sin(self.init_a))
        new_y = self.init_y + (x_odom * math.sin(self.init_a)) + (y_odom * math.cos(self.init_a))
        new_theta = self.init_a + theta_odom

        new_q = quaternion_from_euler(0, 0, new_theta)

        shifted_odom.pose.pose.position.x = new_x
        shifted_odom.pose.pose.position.y = new_y
        shifted_odom.pose.pose.position.z = msg.pose.pose.position.z

        shifted_odom.pose.pose.orientation.x = new_q[0]
        shifted_odom.pose.pose.orientation.y = new_q[1]
        shifted_odom.pose.pose.orientation.z = new_q[2]
        shifted_odom.pose.pose.orientation.w = new_q[3]

        shifted_odom.pose.covariance = msg.pose.covariance
        shifted_odom.twist = msg.twist

        self.pub.publish(shifted_odom)


if __name__ == '__main__':
    try:
        OdomShifter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass