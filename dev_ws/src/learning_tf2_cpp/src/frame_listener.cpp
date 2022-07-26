/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-26 22:53:51
 */

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


class FrameListener : public rclcpp::Node
{
public:
    explicit FrameListener(const std::string & node)
    : Node(node), turtle_spawn_service_ready(false), turtle_spawned(false)
    {
        this->declare_parameter<std::string>("source_frame", "turtle1");
        this->get_parameter("source_frame", source_frame);
        this->declare_parameter<std::string>("target_frame", "turtle2");
        this->get_parameter("target_frame", target_frame);

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        client = this->create_client<turtlesim::srv::Spawn>("spawn");
        pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FrameListener::timer_callback, this));
    }

private:
    bool turtle_spawn_service_ready;
    bool turtle_spawned;
    std::string target_frame;
    std::string source_frame;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    void timer_callback()
    {
        if (turtle_spawn_service_ready)
        {
            if (turtle_spawned)
            {
                geometry_msgs::msg::TransformStamped trans;
                try
                {
                    trans = tf_buffer->lookupTransform(
                            target_frame,
                            source_frame,
                            tf2::TimePointZero
                            );
                }
                catch (tf2::TransformException & ex)
                {
                    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                                 source_frame.c_str(),
                                 target_frame.c_str(),
                                 ex.what());
                    return;
                }

                geometry_msgs::msg::Twist msg;
                static const double scale_rotation_rate = 1.0;
                msg.angular.z = scale_rotation_rate * atan2(trans.transform.translation.y, trans.transform.translation.x);
                static const double scale_forward_rate = 0.5;
                msg.linear.x = scale_forward_rate * sqrt(pow(trans.transform.translation.y, 2) +
                                                         pow(trans.transform.translation.x, 2));
                pub->publish(msg);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                turtle_spawned = true;
            }
        }
        else
        {
            if (client->service_is_ready())
            {
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = target_frame;

                using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future)
                {
                    auto & result = future.get();
                    if (strcmp(result->name.c_str(), "turtle2") == 0)
                    {
                        turtle_spawn_service_ready = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                    }
                };
                auto result = client->async_send_request(request, response_received_callback);
            }
            else    // service ready
            {
                RCLCPP_ERROR(this->get_logger(), "Service is not ready");
            }
        }
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>("frame_listener_node"));
    rclcpp::shutdown();
    return 0;
}
