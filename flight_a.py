import math

import rospy
import cv2
import numpy as np
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge


rospy.init_node('flight')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=2, yaw=float('nan'), speed=1, frame_id='map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


dist = 0  # range
def range_callback(msg):
    global dist
    dist = msg.range

def image_callback(data):
    img = cv2.resize(bridge.imgmsg_to_cv2(data, 'bgr8'), (320, 240))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    cv2.imshow('img', img)
    if cv2.waitKey(1) == ord('q'):
        exit()

rospy.Subscriber('rangefinder/range', Range, range_callback)

subscriber = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
    
navigate_wait(frame_id='body', auto_arm=True)
navigate_wait(0, 0, 1, frame_id='aruco_map')

land_wait()
