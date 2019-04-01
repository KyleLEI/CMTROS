#!/usr/bin/env python2

print "import 0" 


import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon



print "import 1"    


class Kalman_filter(object):
    def __init__(self):
        super(Kalman_filter, self).__init__()
        self.last_measurement = np.array((4, 1), np.float32)
        self.current_measurement = np.array((4, 1), np.float32)
        self.last_predicition = np.zeros((4, 1), np.float32)
        self.current_prediction = np.zeros((4, 1), np.float32)

        self.kalman = cv2.KalmanFilter(4, 4)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.3


    def update(self, Z):
        self.last_measurement = self.current_measurement
        self.last_prediction = self.current_prediction

        self.current_measurement = Z
        self.kalman.correct(self.current_measurement)
        self.current_prediction = self.kalman.predict()




class Subscriber(object):
    def __init__(self):
        super(Subscriber, self).__init__()
        rospy.init_node('kalman_filter_node', anonymous=True)
        self.kalman_filter = Kalman_filter()
        self.pub_kalman = rospy.Publisher('kalman_output', Polygon)

        rospy.Subscriber('/cmt_node/cmt_output', Polygon, self.callback_cmt)
        rospy.spin()

    def callback_cmt(self, data):
        print "received data: ", data
        width = max(abs(data.points[1].x - data.points[0].x), abs(data.points[2].x - data.points[0].x), abs(data.points[3].x - data.points[0].x))
        height = max(abs(data.points[1].y - data.points[0].y), abs(data.points[2].y - data.points[0].y), abs(data.points[3].y - data.points[0].y))
        center_x = min(data.points[0].x, data.points[1].x, data.points[2].x, data.points[3].x) + 0.5 * width
        center_y = min(data.points[0].y, data.points[1].y, data.points[2].y, data.points[3].y) + 0.5 * height

        Z = np.array([[np.float32(center_x)], [np.float32(center_y)], [np.float32(width)], [np.float32(height)]])
        self.kalman_filter.update(Z)

        new_data = Polygon()
        new_data.points = [Point32(),Point32()]
        new_data.points[0].x = self.kalman_filter.current_prediction[0] # center_x
        new_data.points[0].y = self.kalman_filter.current_prediction[1] # center_y
        new_data.points[1].x = self.kalman_filter.current_prediction[2] # width
        new_data.points[1].y = self.kalman_filter.current_prediction[3] # height
        self.pub_kalman.publish(new_data)



if __name__ == '__main__':
    print "main 1"    
    subscriber = Subscriber()
    print "main 2"  
    rospy.spin()




