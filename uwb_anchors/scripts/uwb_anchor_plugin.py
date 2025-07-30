#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import gazebo_msgs.msg
import geometry_msgs.msg
import tf

class UWBAnchorPlugin:
    def __init__(self, anchor_id, namespace="/uwb"):
        self.anchor_id = anchor_id
        self.namespace = namespace
        
        # Инициализация ROS
        rospy.init_node('uwb_anchor_{}_plugin'.format(anchor_id), anonymous=True)
        
        # Публикатор расстояния
        self.distance_pub = rospy.Publisher(
            '{}/anchor{}/distance'.format(namespace, anchor_id),
            Float64,
            queue_size=10
        )
        
        # Подписка на позицию дрона
        rospy.Subscriber(
            '/ground_truth/state',
            gazebo_msgs.msg.ModelStates,
            self.model_states_callback
        )
        
        # Позиция анкора (задаётся в параметрах)
        self.anchor_position = rospy.get_param(
            '~anchor{}_position'.format(anchor_id),
            [0.0, 0.0, 0.0]  # По умолчанию
        )
        
    def model_states_callback(self, msg):
        try:
            # Находим индекс дрона Clover в списке моделей
            drone_index = msg.name.index('clover')
            
            # Получаем позицию дрона
            drone_pos = msg.pose[drone_index].position
            
            # Вычисляем расстояние
            distance = ((drone_pos.x - self.anchor_position[0])**2 +
                       (drone_pos.y - self.anchor_position[1])**2 +
                       (drone_pos.z - self.anchor_position[2])**2)**0.5
            
            # Публикуем расстояние
            self.distance_pub.publish(Float64(distance))
            
        except ValueError:
            # Дрон Clover не найден в списке
            pass

if __name__ == '__main__':
    try:
        # Получаем ID анкора из параметра
        anchor_id = rospy.get_param('~anchor_id', 1)
        
        plugin = UWBAnchorPlugin(anchor_id)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass