#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class UWBAnchorPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Сохраняем указатель на модель
        this->model = _parent;
        
        // Инициализация ROS
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("ROS не инициализирован!");
            return;
        }
        
        // Получаем параметры из SDF
        std::string ns = "/uwb";
        if (_sdf->HasElement("ros"))
        {
            sdf::ElementPtr rosElem = _sdf->GetElement("ros");
            if (rosElem->HasElement("namespace"))
                ns = rosElem->Get<std::string>("namespace");
        }
        
        int anchor_id = 1;
        if (_sdf->HasElement("anchor_id"))
            anchor_id = _sdf->Get<int>("anchor_id");
        
        // Создаем издателя
        std::string topic_name = ns + "/anchor" + std::to_string(anchor_id) + "/distance";
        pub = nh.advertise<std_msgs::Float64>(topic_name, 10);
        
        // Подключаем обработчик обновления мира
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&UWBAnchorPlugin::OnUpdate, this));
    }
    
    void OnUpdate()
    {
        // Здесь должна быть логика вычисления расстояния до объекта
        // Пока просто публикуем тестовое значение
        std_msgs::Float64 msg;
        msg.data = 0.0; // Замените на реальное расстояние
        pub.publish(msg);
    }
    
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle nh;
    ros::Publisher pub;
};

GZ_REGISTER_MODEL_PLUGIN(UWBAnchorPlugin)
}