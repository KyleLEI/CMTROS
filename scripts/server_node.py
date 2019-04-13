#!/usr/bin/env python
import roslib
import sys
import rospy
from dynamic_reconfigure.server import Server
from self_aiming.cfg import tuningConfig


def callback(config, level):
    x_kp = config.x_kp
    x_kd = config.x_kd
    y_kp = config.y_kp
    y_kd = config.y_kd
    z_kp = config.z_kp
    z_kd = config.z_kd
    rospy.loginfo("setting x_kp: %f, x_kd: %f, y_kp:%f, y_kd:%f, z_kp:%f, z_kd:%f",
                  x_kp, x_kd, y_kp, y_kd, z_kp, z_kd)
    return config


if __name__ == "__main__":
    rospy.init_node('server_node')
    srv = Server(tuningConfig, callback)
    rospy.spin()
