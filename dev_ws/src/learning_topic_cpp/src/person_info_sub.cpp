/*
 * @Description: ROS2 topic接收自定义消息 python版本
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-22 13:53:25
 */

#include "rclcpp/rclcpp.hpp"
#include "learning_interface/msg/person_info.hpp"

#include <functional>
#include <memory>

class PersonInfoSub : public rclcpp::Node
{
private:
    rclcpp::Subscription<learning_interface::msg::PersonInfo>::SharedPtr sub;
    void person_callback(const learning_interface::msg::PersonInfo & msg) const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "name: " << msg.name << " | age: " << msg.age << " | id: " << msg.id);
    }

public:
    PersonInfoSub(const std::string & node) : Node(node)
    {
        sub = this->create_subscription<learning_interface::msg::PersonInfo>("person_info_topic", 10,
                                                                             std::bind(&PersonInfoSub::person_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonInfoSub>("person_info_sub_node"));
    rclcpp::shutdown();
    return 0;
}
