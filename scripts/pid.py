#!/usr/bin/env python2
from __future__ import print_function
from swarmtal_msgs.msg import drone_onboard_command
from geometry_msgs.msg import Polygon
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

        self.initialized = False
        self.sub = rospy.Subscriber("/kalman_output",Polygon,self.callback)
        self.pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)

    def callback(self,data):
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
        x = self.x_kp*self.cur_x_err + self.x_kd*(self.cur_x_err-self.last_x_err)
        self.last_x_err = self.cur_x_err
        self.cur_z_err = self.image_height/2 - data.points[0].y
        z = self.z_kp*self.cur_z_err + self.z_kd*(self.cur_z_err-self.last_z_err)
        self.last_z_err = self.cur_z_err
        self.cur_y_err = self.bounding_box_size - math.sqrt(data.points[1].x*data.points[1].x+data.points[1].y*data.points[1].y)
        y = self.y_kp*self.cur_y_err + self.y_kd*(self.cur_y_err-self.last_y_err)
        self.last_y_err = self.cur_y_err
        cmd.param1 = int(x)
        cmd.param2 = int(y)
        cmd.param3 = int(z)
        self.pub.publish(cmd)

def main():
    rospy.init_node('pid_node')
    PID = pid()
    rospy.spin()
    

if __name__ == "__main__":
    main()
