/*
 * @Description: ROS2 service服务器 接收自定义消息 cpp版本
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-22 20:37:15
 */

#include "rclcpp/rclcpp.hpp"
#include "learning_interface/srv/person_info.hpp"


class PersonInfoServer: public rclcpp::Node
{
private:
    rclcpp::Service<learning_interface::srv::PersonInfo>::SharedPtr server;

    void person_callback(const learning_interface::srv::PersonInfo::Request::SharedPtr & request,
                         const learning_interface::srv::PersonInfo::Response::SharedPtr & response)
    {
        if (request->name == "HTY")
        {
            response->age = 21;
            response->id = 3190105708;
        }
        else if (request->name == "ZXA")
        {
            response->age = 20;
            response->id = 3190105598;
        }
        response->message = "Get person info!";

        RCLCPP_INFO_STREAM(this->get_logger(), "name: " << request->name << " | age: " << (int) response->age << " | id: " << response->id);
    }

public:
    explicit PersonInfoServer(const std::string & node) : Node(node)
    {
        server = this->create_service<learning_interface::srv::PersonInfo>
                ("person_info_service",
                 std::bind(&PersonInfoServer::person_callback, this,
                           std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO_STREAM(this->get_logger(), "Service started");
    }
};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
//    // 把节点的执行器变成多线程执行器, 避免死锁
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(std::make_shared<PersonInfoServer>("person_info_server_node"));
//    executor.spin();
    rclcpp::spin(std::make_shared<PersonInfoServer>("person_info_server_node"));
    rclcpp::shutdown();
    return 0;
}