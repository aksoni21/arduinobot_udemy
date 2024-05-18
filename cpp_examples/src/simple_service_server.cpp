#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduino_bot_msgs/srv/add_two_ints.hpp>

class SimpleServiceServer : public rclcpp::Node
{
    public:
        SimpleServiceServer(): Node("simple_service_server")
        {

        }

    private:
        rclcpp::Service<>::SharedPtr service_; 
}