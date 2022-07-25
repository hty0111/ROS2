/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-23 17:20:43
 */

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "learning_interface/action/fibonacci.hpp"


namespace learning_action_cpp
{

class FibonacciServer : public rclcpp::Node
{
public:
    using Fibonacci = learning_interface::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    explicit FibonacciServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("fibonacci_server_node", options)

    {
        using namespace std::placeholders;

        this->action_server = rclcpp_action::create_server<Fibonacci>(
                this,
                "fibonacci",
                std::bind(&FibonacciServer::handle_goal, this, _1, _2),
                std::bind(&FibonacciServer::handle_cancel, this, _1),
                std::bind(&FibonacciServer::handle_accepted, this, _1)
                );
        RCLCPP_INFO(this->get_logger(), "Server started");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const Fibonacci::Goal> & goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> & goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> & goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&FibonacciServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> & goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; i < goal->order && rclcpp::ok(); i++)
        {
            // check if there is a cancel request
            if (goal_handle->is_canceling())
            {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            // update sequence
            sequence.push_back(sequence[i-1] + sequence[i]);
            // publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }

        // check if goal is done
        if (rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(learning_action_cpp::FibonacciServer)