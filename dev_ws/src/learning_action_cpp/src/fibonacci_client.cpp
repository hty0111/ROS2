/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-23 17:21:02
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "learning_interface/action/fibonacci.hpp"


namespace learning_action_cpp
{

class FibonacciClient : public rclcpp::Node
{
public:
    using Fibonacci = learning_interface::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciClient(const rclcpp::NodeOptions & options) : Node("fibonacci_client_node", options)
    {
        this->action_client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
        this->timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FibonacciClient::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer->cancel();

        if (!this->action_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server is not available");
            rclcpp::shutdown();
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&FibonacciClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&FibonacciClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&FibonacciClient::result_callback, this, _1);
        this->action_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client;
    rclcpp::TimerBase::SharedPtr timer;

    void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    }

    void feedback_callback(const GoalHandleFibonacci::SharedPtr&, const std::shared_ptr<const Fibonacci::Feedback>& feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was cancelled");
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
    }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(learning_action_cpp::FibonacciClient)
