#!/usr/bin/env python2
from __future__ import print_function
from swarmtal_msgs.msg import drone_onboard_command
from geometry_msgs.msg import Polygon
import rospy
import math

x_kp = 0
cur_x_err = 0
last_x_err = 0
x_kd = 0
y_kp = 0
cur_y_err = 0
last_y_err = 0
bounding_box_size = 0
y_kd = 0
z_kp = 0
cur_z_err = 0
last_z_err = 0
z_kd = 0

image_width = 320
image_height = 240

initialized = false

def callback(data):
    if not initialized:
        initialized = true
        bounding_box_size = math.sqrt(data.points[1].x*data.points[1].x+data.points[1].y*data.points[1].y)       
 
    cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
    if rospy.has_param('/server_node/y_kp'):
        y_kp = rospy.get_param('/server_node/y_kp')
        y_kd = rospy.get_param('/server_node/y_kd')
        z_kp = rospy.get_param('/server_node/z_kp')
        z_kd = rospy.get_param('/server_node/z_kd')
        x_kp = rospy.get_param('/server_node/x_kp')
        x_kd = rospy.get_param('/server_node/x_kd')

    cmd.param4 = 666666
    cur_x_err = image_width/2 - data.points[0].x
    x = x_kp*cur_x_err + x_kd*(cur_x_err-last_x_err)
    last_x_err = cur_x_err
    cur_z_err = image_height/2 - data.points[0].y
    z = z_kp*cur_z_err + z_kd*(cur_z_err-last_z_err)
    last_z_err = cur_z_err
    cur_y_err = bounding_box_size - math.sqrt(data.points[1].x*data.points[1].x+data.points[1].y*data.points[1].y)
    y = y_kp*cur_y_err + y_kd*(cur_y_err-last_y_err)
    last_y_err = cur_y_err
    cmd.param1 = int(x*10000)
    cmd.param2 = int(y*10000)
    cmd.param3 = int(z*10000)
    pub.publish(cmd)
    rate.sleep()

def main():
    rospy.init_node('PID NODE', anonymous=True)
    sub = rospy.Subscriber("/kalman_output",Polygon,callback)
    rospy.spin()
    

if __name__ == "__main__":
    main()
