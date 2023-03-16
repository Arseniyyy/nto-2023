import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range


rospy.init_node('flight')


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

navigate_wait(z=0.5, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=1, z=0.5, speed=0.8, yaw=3.14159)
#navigate_wait(x=1.5, y=1.3, z=1.25, frame_id='aruco_map')
navigate_wait(x=0, y=4, z=0.5, speed=0.8, yaw=3.14159)


#1-2 стены
rast = 0
dist = rospy.wait_for_message('rangefinder/range', Range).range
a = dist
while round(dist) == round(a):
    navigate_wait(x=0.1, frame_id='body')
    dist = rospy.wait_for_message('rangefinder/range', Range).range
rast = get_telemetry(frame_id='aruco_map').x
dist = rospy.wait_for_message('rangefinder/range', Range).range
b = dist
rast1 = get_telemetry(frame_id='aruco_map').x
wall1 = rast1 - rast
wall2 = b - a

#3-4 стены
dist = rospy.wait_for_message('rangefinder/range', Range).range
while round(dist) == round(b):
    dist = rospy.wait_for_message('rangefinder/range', Range).rang
    navigate_wait(x=0.1, frame_id='body')
a = rospy.wait_for_message('rangefinder/range', Range).range
rast = get_telemetry(frame_id='aruco_map').x
wall3 = rast - rast1
wall4 = b - a

#5-6 стены
while round(dist) == round(a):
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    navigate_wait(x=0.1, frame_id='body')
rast = get_telemetry(frame_id='aruco_map').x
b = rospy.wait_for_message('rangefinder/range', Range).range
wall5 = rast1 - rast
wall6 = b - a

#7-8 стены
dist = rospy.wait_for_message('rangefinder/range', Range).range
while round(dist) == round(b):
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    navigate_wait(x=0.1, frame_id='body')
a = rospy.wait_for_message('rangefinder/range', Range).range
rast = get_telemetry(frame_id='aruco_map').x
wall7 = rast - rast1
wall8 = b - a

#9-10 стены
while round(dist) == round(a):
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    navigate_wait(x=0.1, frame_id='body')
b = rospy.wait_for_message('rangefinder/range', Range).range
rast1 = get_telemetry(frame_id='aruco_map').x
wall9 = rast1 - rast
wall10 = b - a

navigate_wait(x=0, y=4, z=0.5, frame_id='aruco_map')

print('Wall 1:', round(wall1))
print('Wall 2:', round(wall2))
print('Wall 3:', round(wall3))
print('Wall 4:', round(wall4))
print('Wall 5:', round(wall5))
print('Wall 6:', round(wall6))
print('Wall 7:', round(wall7))
print('Wall 8:', round(wall8))
print('Wall 9:', round(wall9))
print('Wall 10:', round(wall10))