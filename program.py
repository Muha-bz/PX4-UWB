#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

class CloverUWBSquare2D:
    def __init__(self):
        rospy.init_node('clover_uwb_square_2d')

        # Конфигурация UWB анкоров (X,Y) в метрах
        self.anchors = {
            'anchor1': np.array([0.0, 0.0]),
            'anchor2': np.array([3.0, 0.0]),
            'anchor3': np.array([0.0, 3.0])
        }

        # Текущие расстояния до анкоров
        self.distances = {k: None for k in self.anchors}
        
        # Точки квадратного маршрута [X,Y,Z] (Z постоянный)
        self.waypoints = [
            [1.0, 1.0, 1.5],
            [2.0, 1.0, 1.5],
            [2.0, 2.0, 1.5],
            [1.0, 2.0, 1.5],
            [1.0, 1.0, 1.5]  # Возврат в начальную точку
        ]
        self.current_waypoint = 0
        self.position_tolerance = 0.2  # Точность позиционирования (м)

        # Параметры управления
        self.xy_speed = 0.4  # Скорость движения XY (м/с)
        self.z_speed = 0.4   # Скорость по Z (м/с)
        self.hover_height = 1.5  # Высота полета (м)

        # ROS сервисы Clover
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', Trigger)

        # Подписки на UWB (теперь используем Float64 вместо Range)
        rospy.Subscriber('/uwb/anchor1/distance', Float64, lambda msg: self.uwb_callback(msg, 'anchor1'))
        rospy.Subscriber('/uwb/anchor2/distance', Float64, lambda msg: self.uwb_callback(msg, 'anchor2'))
        rospy.Subscriber('/uwb/anchor3/distance', Float64, lambda msg: self.uwb_callback(msg, 'anchor3'))

        # Таймер навигации
        rospy.Timer(rospy.Duration(0.1), self.navigation_loop)

    def uwb_callback(self, msg, anchor_name):
        # Для Float64 расстояние хранится прямо в msg.data
        self.distances[anchor_name] = msg.data

    def calculate_2d_position(self):
        """2D трилатерация (возвращает [x,y] или None)"""
        if None in self.distances.values():
            return None

        A = []
        b = []
        anchor0 = self.anchors['anchor1']
        d0 = self.distances['anchor1']

        for name in ['anchor2', 'anchor3']:
            Ai = 2 * (self.anchors[name] - anchor0)
            bi = np.sum(anchor0**2) - np.sum(self.anchors[name]**2) \
                 + self.distances[name]**2 - d0**2
            A.append(Ai)
            b.append(bi)

        try:
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return None

    def navigation_loop(self, event):
        # Получаем текущую позицию
        position_2d = self.calculate_2d_position()
        if position_2d is None:
            rospy.logwarn("Не могу определить позицию! Ожидание данных UWB...")
            return

        # Текущая точка маршрута
        target = self.waypoints[self.current_waypoint]
        
        # Вычисляем расстояние до цели (только по XY)
        distance = np.linalg.norm(np.array(target[:2]) - position_2d)
        
        # Проверяем достижение точки
        if distance < self.position_tolerance:
            self.current_waypoint += 1
            if self.current_waypoint >= len(self.waypoints):
                self.complete_mission()
                return
            rospy.loginfo(f"Достигнута точка {self.current_waypoint}/{len(self.waypoints)-1}")

        # Движение к точке
        self.navigate(
            x=float(target[0]),
            y=float(target[1]),
            z=float(target[2]),
            speed=self.xy_speed,
            frame_id='aruco_map',
            auto_arm=True
        )

    def complete_mission(self):
        rospy.loginfo("Маршрут завершен! Посадка...")
        self.land()
        rospy.signal_shutdown("Миссия выполнена")

    def run(self):
        # Взлет
        rospy.loginfo("Взлет на высоту {} м...".format(self.hover_height))
        self.navigate(z=self.hover_height, speed=self.z_speed, frame_id='body', auto_arm=True)
        rospy.sleep(5)
        
        # Основной цикл
        rospy.spin()

if __name__ == '__main__':
    navigator = CloverUWBSquare2D()
    try:
        navigator.run()
    except rospy.ROSInterruptException:
        pass