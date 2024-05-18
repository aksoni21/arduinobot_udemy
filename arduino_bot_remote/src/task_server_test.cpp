
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// #include <arduino_bot_msgs/action/arduinobot_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
// #include <thread>

namespace arduino_bot_remote
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("task_server", options)
        {
            using namespace std::placeholders;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pre ---Action server ready.");
        }

    private:
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(arduino_bot_remote::TaskServer)
// RCLCPP_COMPONENTS_REGISTER_NODE(::TaskServer)




