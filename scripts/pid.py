#!/usr/bin/env python2
from __future__ import print_function
from swarmtal_msgs.msg import drone_onboard_command
from geometry_msgs.msg import Polygon
from nav_msgs.msg import Odometry
import rospy
import math

class pid(object):
    def __init__(self):
        self.x_kp = 0
        self.cur_x_err = 0
        self.last_x_err = 0
        self.x_kd = 0
        self.y_kp = 0
        self.cur_y_err = 0
        self.last_y_err = 0
        self.bounding_box_size = 0
        self.y_kd = 0
        self.z_kp = 0
        self.cur_z_err = 0
        self.last_z_err = 0
        self.z_kd = 0

        self.image_width = 640
        self.image_height = 480

        self.vicon_x = 0
        self.vicon_y = 0
        self.vicon_z = 0

        self.initialized = False
        self.sub_kalman = rospy.Subscriber("/kalman_output",Polygon,self.kalman_callback)
        self.sub_vicon = rospy.Subscriber("/uwb_vicon_odom", Odometry, self.vicon_callback)
        self.pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)

    def vicon_callback(self,data):
        self.vicon_x = data.pose.pose.position.x
        self.vicon_y = data.pose.pose.position.y
        self.vicon_z = data.pose.pose.position.z


    def kalman_callback(self,data):
        if not self.initialized:
            self.initialized = True
            self.bounding_box_size = math.sqrt(data.points[1].x*data.points[1].x+data.points[1].y*data.points[1].y)
        cmd = drone_onboard_command()
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        if rospy.has_param('/server_node/y_kp'):
            self.y_kp = rospy.get_param('/server_node/y_kp')
            self.y_kd = rospy.get_param('/server_node/y_kd')
            self.z_kp = rospy.get_param('/server_node/z_kp')
            self.z_kd = rospy.get_param('/server_node/z_kd')
            self.x_kp = rospy.get_param('/server_node/x_kp')
            self.x_kd = rospy.get_param('/server_node/x_kd')

        cmd.param4 = 666666

        self.cur_x_err = self.image_width/2 - data.points[0].x
        cameara_delta_x = self.x_kp*self.cur_x_err + self.x_kd*(self.cur_x_err-self.last_x_err)
        self.last_x_err = self.cur_x_err

        self.cur_y_err = self.image_height/2 - data.points[0].y
        cameara_delta_y = self.y_kp*self.cur_y_err + self.y_kd*(self.cur_y_err-self.last_y_err)
        self.last_y_err = self.cur_y_err

        self.cur_z_err = self.bounding_box_size - math.sqrt(data.points[1].x*data.points[1].x+data.points[1].y*data.points[1].y)
        cameara_delta_z = self.z_kp*self.cur_z_err + self.z_kd*(self.cur_z_err-self.last_z_err)
        self.last_z_err = self.cur_z_err

        # camera x -> vicon y
        # camera y -> vicon -z
        # camera z -> vicon -x
        vicon_command_x = self.vicon_x + cameara_delta_y
        vicon_command_y = self.vicon_y - cameara_delta_z
        vicon_command_z = self.vicon_z - cameara_delta_x

        cmd.param1 = int(vicon_command_x)
        cmd.param2 = int(vicon_command_y)
        cmd.param3 = int(vicon_command_z)
        self.pub.publish(cmd)

def main():
    rospy.init_node('pid_node')
    PID = pid()
    rospy.spin()


if __name__ == "__main__":
    main()
