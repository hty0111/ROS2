/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-25 14:54:12
 */

#include "rclcpp/rclcpp.hpp"

class Param : public rclcpp::Node
{
public:
    explicit Param(const std::string & node) : Node(node)
    {
        this->declare_parameter<std::string>("learn_param", "world");
        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Param::respond, this));
    }
    void respond()
    {
        this->get_parameter("learn_param", param_string);
        RCLCPP_INFO(this->get_logger(), "Hello %s", param_string.c_str());
    }

private:
    std::string param_string;
    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Param>("param_node"));
    rclcpp::shutdown();
    return 0;
}
