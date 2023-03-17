#Импорт библиотек
import math
from sensor_msgs.msg import Range
#Создание объектов-прокси для использование сервисов
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):#Функция для полета в точку и ожидание окончания полета
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def land_wait():#Функция для посадки и ожидания окончания посадки
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


navigate_wait(z=0.5, frame_id='body', auto_arm=True)#Взлёт
#Перемещение к точке входа
navigate_wait(x=0, y=1, z=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=4, z=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=4, z=0.5, speed=0.5, yaw=3.14159)
navigate_wait(x=0, y=4, z=0.5, frame_id='aruco_map')


#Распознование 1-2 стен
rast = 0
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
a = dist
while round(dist) == round(a):#Поиск стены
    navigate_wait(x=0.1, frame_id='body')#Перемещение на 0.1м по x
    dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
rast = get_telemetry(frame_id='aruco_map').x#Получение кординат
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
b = dist
rast1 = get_telemetry(frame_id='aruco_map').x#Получение кординат
wall1 = rast1 - rast#размер 1 стены
wall2 = b - a#размер 2 стены

#Распознование 3-4 стен
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
while round(dist) == round(b):#Поиск стены
    dist = rospy.wait_for_message('rangefinder/range', Range).rang#Получение данных с лазерного дальномера
    navigate_wait(x=0.1, frame_id='body')#Перемещение на 0.1м по x
a = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
rast = get_telemetry(frame_id='aruco_map').x#Получение кординат
wall3 = rast - rast1#размер 3 стены
wall4 = b - a#размер 4 стены

#Распознование 5-6 стен
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
while round(dist) == round(a):#Поиск стены
    dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
    navigate_wait(x=0.1, frame_id='body')#Перемещение на 0.1м по x
rast = get_telemetry(frame_id='aruco_map').x#Получение кординат
b = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
wall5 = rast1 - rast#размер 5 стены
wall6 = b - a#размер 6 стены

#Распознование 7-8 стен
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
while round(dist) == round(b):#Поиск стены
    dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
    navigate_wait(x=0.1, frame_id='body')#Перемещение на 0.1м по x
a = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
rast = get_telemetry(frame_id='aruco_map').x#Получение кординат
wall7 = rast - rast1#размер 7 стены
wall8 = b - a#размер 8 стены

#Распознование 9-10 стен
dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
while round(dist) == round(a):#Поиск стены
    dist = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
    navigate_wait(x=0.1, frame_id='body')#Перемещение на 0.1м по x
b = rospy.wait_for_message('rangefinder/range', Range).range#Получение данных с лазерного дальномера
rast1 = get_telemetry(frame_id='aruco_map').x#Получение кординат
wall9 = rast1 - rast#размер 9 стены
wall10 = b - a#размер 10 стены

#Вывод
print('Wall 1:', round(wall1) / 100)
print('Wall 2:', round(wall2) / 100)
print('Wall 3:', round(wall3) / 100)
print('Wall 4:', round(wall4) / 100)
print('Wall 5:', round(wall5) / 100)
print('Wall 6:', round(wall6) / 100)
print('Wall 7:', round(wall7) / 100)
print('Wall 8:', round(wall8) / 100)
print('Wall 9:', round(wall9) / 100)
print('Wall 10:', round(wall10) / 100)
#Возвращение в зону взлёта
navigate_wait(x=0, y=4, z=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=1, z=0.5, frame_id='aruco_map')
navigate_wait(x=1.5, y=0.5, z=0.5, frame_id='aruco_map')
land_wait()#Посадка