/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-26 22:19:03
 */

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"


class FrameBroadcaster : public rclcpp::Node
{
public:
    explicit FrameBroadcaster(const std::string & node) : Node(node)
    {
        using namespace std::placeholders;

        this->declare_parameter<std::string>("turtlename", "turtle1");
        this->get_parameter("turtlename", turtlename);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;
        stream << "/" << turtlename.c_str() << "/pose";
        std::string topic_name = stream.str();

        sub = this->create_subscription<turtlesim::msg::Pose>(
                topic_name, 10,
                std::bind(&FrameBroadcaster::pose_callback, this, _1)
                );
    }

private:
    std::string turtlename;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub;
    void pose_callback(const turtlesim::msg::Pose & msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped trans;

        trans.header.stamp = now;
        trans.header.frame_id = "world";
        trans.child_frame_id = turtlename.c_str();

        trans.transform.translation.x = msg.x;
        trans.transform.translation.y = msg.y;
        trans.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg.theta);
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(trans);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameBroadcaster>("tf2_broadcaster_node"));
    rclcpp::shutdown();
    return 0;
}
