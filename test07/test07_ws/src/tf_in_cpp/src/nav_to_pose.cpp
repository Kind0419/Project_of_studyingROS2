#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigationAction = nav2_msgs::action::NavigateToPose;

class NavToPoseClient : public rclcpp::Node
{
public:
    using NavigationActionClient = rclcpp_action::Client<NavigationAction>;
    using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

    NavToPoseClient() : Node("nav_to_pose_client")
    {
        action_client_ = rclcpp_action::create_client<NavigationAction>(this, "navigate_to_pose");
    }

    void sendGoal()
    {
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(get_logger(), "等待Action服务上线。");
        }

        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 2.0f;
        goal_msg.pose.pose.position.y = 2.0f;

        auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](NavigationActionGoalHandle::SharedPtr goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
            }
        };

        send_goal_options.feedback_callback = [this](NavigationActionGoalHandle::SharedPtr rcl_action_goal_handle,
                                                     const std::shared_ptr<const NavigationAction::Feedback> feedback)
        {
            (void)rcl_action_goal_handle;
            RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f", feedback->distance_remaining);
        };

        send_goal_options.result_callback =
            [this](const NavigationActionGoalHandle::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "处理成功！");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    node->sendGoal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
