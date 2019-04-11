import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('img_cvt_node',anonymous=True)
image_pub = rospy.Publisher('/camera/image_raw',Image)
bridge = CvBridge()

cap = cv2.VideoCapture()
cap.open(0)
while not rospy.is_shutdown():
    ret,img_cv = cap.read()
    img_cv = img_cv[:,:,1]
    #cv2.cvtColor(img_cv,img_cv,cv2.COLOR_BGR2GRAY)
    img_ros = bridge.cv2_to_imgmsg(img_cv,'mono8')
    image_pub.publish(img_ros)

    cv2.imshow('Image',img_cv)
    k = cv2.waitKey(10)

