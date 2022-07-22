/*
 * @Description: ROS2 topic发送自定义消息 cpp版本
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-22 13:49:06
 */


#include "rclcpp/rclcpp.hpp"
#include "learning_interface/msg/person_info.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PersonInfoPub : public rclcpp::Node
{
private:
    size_t count;
    rclcpp::Publisher<learning_interface::msg::PersonInfo>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    void person_callback()
    {
        auto msg = learning_interface::msg::PersonInfo();
        msg.name = "HTY";
        msg.age = 21;
        msg.id = 3190105708;
        pub->publish(msg);
        RCLCPP_INFO_STREAM(this->get_logger(), "Loop: " << count << " | name: " << msg.name << " | age: " << msg.age << " | id: " << msg.id);
        count++;
    }

public:
    explicit PersonInfoPub(const std::string & node) : Node(node), count(1)
    {
        pub = this->create_publisher<learning_interface::msg::PersonInfo>("person_info_topic", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(500), [this] { person_callback(); });
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PersonInfoPub>("person_info_pub_node"));
    rclcpp::shutdown();
    return 0;
}
