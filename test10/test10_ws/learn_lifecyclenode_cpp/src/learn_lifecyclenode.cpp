// 引入必要的ROS 2头文件
#include "rclcpp/rclcpp.hpp"  // ROS 2 C++客户端库的核心功能
#include "rclcpp_lifecycle/lifecycle_node.hpp"  // 生命周期节点相关功能

// 为回调返回类型定义一个简短的别名
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 定义一个继承自LifecycleNode的生命周期节点类
class LearnLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    // 构造函数
    LearnLifeCycleNode() : rclcpp_lifecycle::LifecycleNode("lifecyclenode")
    {
        timer_period_ = 1.0;  // 初始化定时器周期为1秒
        timer_ = nullptr;     // 初始化定时器指针为空
        // 打印节点创建信息
        RCLCPP_INFO(get_logger(), "%s:已创建", get_name());
    }
    
    // 配置回调函数 - 当节点从"未配置"状态转移到"非活跃"状态时调用
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
    {
        (void)state;  // 显式忽略未使用的参数，避免编译器警告
        timer_period_ = 1.0;  // 设置定时器周期
        // 打印配置信息
        RCLCPP_INFO(get_logger(), "on_configure():配置周期 timer_period");
        // 返回成功，表示配置完成
        return CallbackReturn::SUCCESS;
    }
    
    // 激活回调函数 - 当节点从"非活跃"状态转移到"活跃"状态时调用
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
    {
        (void)state;  // 显式忽略未使用的参数
        // 创建一个定时器，每秒触发一次
        timer_ = create_wall_timer(
            std::chrono::seconds(static_cast<int>(timer_period_)),
            [this]() { RCLCPP_INFO(get_logger(), "定时器输出进行中..."); }
        );
        // 打印激活信息
        RCLCPP_INFO(get_logger(), "on_activate():处理激活指令，创建定时器");
        // 返回成功，表示激活完成
        return CallbackReturn::SUCCESS;
    }
    
    // 停用回调函数 - 当节点从"活跃"状态转移到"非活跃"状态时调用
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
    {
        (void)state;  // 显式忽略未使用的参数
        timer_.reset();  // 重置(停止)定时器
        // 打印停用信息
        RCLCPP_INFO(get_logger(), "on_deactivate():处理失活指令，停止定时器");
        // 返回成功，表示停用完成
        return CallbackReturn::SUCCESS;
    }
    
    // 关闭回调函数 - 当节点被关闭时调用
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
    {
        (void)state;  // 显式忽略未使用的参数
        timer_.reset();  // 重置(停止)定时器
        // 打印关闭信息
        RCLCPP_INFO(get_logger(), "on_shutdown():处理关闭指令");
        // 返回成功，表示关闭完成
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;  // 定时器共享指针
    double timer_period_;  // 定时器周期(秒)
};

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // 初始化ROS 2
    // 创建生命周期节点实例
    auto node = std::make_shared<LearnLifeCycleNode>();
    // 旋转节点，使其保持运行并处理回调
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();  // 关闭ROS 2
    return 0;
}
