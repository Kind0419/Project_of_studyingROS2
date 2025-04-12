#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

class TFListener:public rclcpp::Node
{
    public:
    TFListener():Node("tf2_listener")
    {
        buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_=this->create_wall_timer(5s,std::bind(&TFListener::getTransform,this));

    }
    void getTransform()
    {
        try
        {
            const auto tf=buffer_->lookupTransform(
                "map","base_footprint",this->get_clock()->now(),
                rclcpp::Duration::from_seconds(1.0f));

            // 转换结果
                const auto &translation=tf.transform.translation;
                const auto &rotation=tf.transform.rotation;
                double yaw,pitch,roll;
                tf2::getEulerYPR(rotation,yaw,pitch,roll);
                RCLCPP_INFO(get_logger(),"平移：（%f, %f, %f)",translation.x,translation.y,translation.z);
                RCLCPP_INFO(get_logger(),"旋转：（%f, %f, %f)",roll,pitch,yaw);
        }
        catch(tf2::TransformException &ex)//异常情况
        {
            RCLCPP_WARN(get_logger(),"不能变换坐标，原因：%s",ex.what());
        }
    }
    private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}