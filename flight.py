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

image_publisher = rospy.Publisher('/image_changed', Image, queue_size=10)
rate = rospy.Rate(1)


def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.4, frame_id='map', auto_arm=False, tolerance=0.2):
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


def fly():
    navigate_wait(frame_id='body', auto_arm=True)
    telemetry = get_telemetry(frame_id='aruco_map')

    navigate_wait(x=0, y=4, frame_id='aruco_map')
    navigate_wait(x=1, y=3, frame_id='aruco_map')
    navigate_wait(telemetry.x, telemetry.y, frame_id='aruco_map')


def takeoff_liftoff():
    navigate_wait(frame_id='body', auto_arm=True)
    telemetry = get_telemetry(frame_id='aruco_map')
    navigate_wait(telemetry.x, telemetry.y, frame_id='aruco_map')


def fly_to_upper_right_corner():
    """Функция полёта в правый верхний угол карты. Используется для тестирования полёта коптера по аруко-маркерам"""
    navigate_wait(frame_id='body', auto_arm=True)
    telemetry = get_telemetry(frame_id='aruco_map')

    navigate_wait(x=0, y=1, frame_id='aruco_map')
    navigate_wait(x=0, y=3, frame_id='aruco_map')
    navigate_wait(x=1, y=3, frame_id='aruco_map')
    navigate_wait(x=1, y=4, frame_id='aruco_map')
    navigate_wait(x=4, y=4, frame_id='aruco_map')
    navigate_wait(x=4, y=4, frame_id='aruco_map')
    navigate_wait(x=4, y=1, frame_id='aruco_map')
    navigate_wait(x=4, y=4, frame_id='aruco_map')
    navigate_wait(x=7, y=4, frame_id='aruco_map')

    navigate_wait(x=0, y=4, frame_id='aruco_map')
    navigate_wait(x=0, y=1, frame_id='aruco_map')
    navigate_wait(x=telemetry.x, y=telemetry.y, frame_id='aruco_map')


def image_callback(data):
    """Функция распознавания пожаров"""
    img = cv2.resize(bridge.imgmsg_to_cv2(data, 'bgr8'), (320, 240))

    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    image_publisher.publish(bridge.cv2_to_imgmsg(img))
    rate.sleep()

    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_red = np.array([0, 50, 50])
    # upper_red = np.array([10, 255, 255])
    # mask1 = cv2.inRange(hsv, lower_red, upper_red)

    # lower_red = np.array([170, 50, 50])
    # upper_red = np.array([180, 255, 255])
    # mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # sorted_contours = sorted(contours, key=cv2.contourArea, reverse=False)

    # try:
    #     # M = cv2.moments(sorted_contours[0])
    #     x, y, w, h = cv2.boundingRect(sorted_contours[0])
    #     x_center = x + w / 2
    #     y_center = y + w / 2

    #     if 200 < x_center < 215 and 100 < y_center < 160:
    #         telem = get_telemetry(frame_id='aruco_map')
    #         print('fire detected', telem.x, telem.y)

    #     # print(x_center, y_center)
    # except:
    #     pass

# rospy.Subscriber('rangefinder/range', Range, range_callback)  # подписка на топик с дальномером

rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)  

fly_to_upper_right_corner()

land_wait()
