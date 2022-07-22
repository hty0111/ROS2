/*
 * @Description: ROS2 service客户端 发送自定义消息 cpp版本
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-22 20:38:40
 */

#include "rclcpp/rclcpp.hpp"
#include "learning_interface/srv/person_info.hpp"


class PersonInfoClient : public rclcpp::Node
{
private:
    rclcpp::Client<learning_interface::srv::PersonInfo>::SharedPtr client;
    void person_info_callback(rclcpp::Client<learning_interface::srv::PersonInfo>::SharedFuture response)
    {
        const auto & result = response.get();
        RCLCPP_INFO_STREAM(this->get_logger(), result->message << std::endl <<
        "age: " << (int) result->age << " id: " << result->id);
    }

public:
    explicit PersonInfoClient(const std::string & node) : rclcpp::Node(node)
    {
        client = this->create_client<learning_interface::srv::PersonInfo>("person_info_service");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for service started...");
        }
    }

    void send_person_info(int argc, char* argv[])
    {
        auto request = std::make_shared<learning_interface::srv::PersonInfo::Request>();
        request->name = argv[1];
        client->async_send_request(request, std::bind(&PersonInfoClient::person_info_callback, this, std::placeholders::_1));
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto person_info_client = std::make_shared<PersonInfoClient>("person_info_client_node");
    person_info_client->send_person_info(argc, argv);
    rclcpp::spin(person_info_client);
    rclcpp::shutdown();
    return 0;
}