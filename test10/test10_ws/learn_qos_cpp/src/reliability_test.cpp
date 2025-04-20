#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomPublisherSubscriber:public rclcpp::Node
{
    public:
    OdomPublisherSubscriber():Node("odom_publisher_subscriber")
    {
        //创建发布者并且设置QoS为sensor
        odom_publisher_=this->create_publisher<nav_msgs::msg::Odometry>(
            "odom",rclcpp::SensorDataQoS()
        );
        // 创建订阅者(默认QoS配置)队列深度设置为5
        odom_subscription_=this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",5,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                (void)msg;
                RCLCPP_INFO(this->get_logger(),"收到里程信息");
            }
        );
        // 创建一个1s的定时器，并指定回调函数
        timer_=this->create_wall_timer(std::chrono::seconds(1),[this]()
        {
        odom_publisher_->publish(nav_msgs::msg::Odometry());
        });

    }
    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto odom_node=std::make_shared<OdomPublisherSubscriber>();
    rclcpp::spin(odom_node);
    rclcpp::shutdown();
    return 0;
}