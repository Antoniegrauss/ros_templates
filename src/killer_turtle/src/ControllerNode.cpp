#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtle_msgs/msg/turtle.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/kill.hpp"

#include "../include/killer_turtle/Position.hpp"

using Pose = geometry_msgs::msg::Pose2D;
using Twist = geometry_msgs::msg::Twist;
using Turtle = turtle_msgs::msg::Turtle;

// Subscribes to alive_turtles
// Store all alive turtles
// Once in a while:
    // teleport turtle to target (service call to /turtlesim TeleportAbsolute)
    // kill target (service call to /spawner KillTurtle)
class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : Node("controller_node")
    {
        declare_parameter("turtle_name", "turtle1");
        turtle_name_ = get_parameter("turtle_name").as_string();

        declare_parameter("kill_countdown", 2);
        const int kill_countdown{get_parameter("kill_countdown").as_int()};

        targets_subscription_ = create_subscription<Turtle>("new_turtle", 10, [this](const Turtle::SharedPtr turtle) {
            targets_.push_back(*turtle);
        });

        timer_ = create_wall_timer(std::chrono::seconds(kill_countdown), [this]() {
            if (! targets_.empty()) {
                killATurtle();
                std::string message {"Killing a turtle, named: " + targets_.front().name};
                RCLCPP_INFO(get_logger(), message.c_str());
            }
        });
    }

private:
    rclcpp::Subscription<Turtle>::SharedPtr targets_subscription_;
    std::string turtle_name_;
    std::vector<Turtle> targets_;
    rclcpp::TimerBase::SharedPtr timer_;

    void killATurtle() {
        const Turtle turtle_to_kill {targets_.front()};
        targets_.erase(targets_.begin());

        // Teleport the turtle to the kill position
        Position target_position{turtle_to_kill.pose.x, turtle_to_kill.pose.y};
        teleportTurtle(target_position);

        killTurtle(turtle_to_kill.name);
    }

    void teleportTurtle(const Position& position) {
        using Teleport = turtlesim::srv::TeleportAbsolute;

        auto request = std::make_shared<Teleport::Request>();
        request->x = position.x;
        request->y = position.y;
        request->theta = 0;

        const std::string teleport_service_name{turtle_name_ + "/teleport_absolute"};
        auto client = create_client<Teleport>(teleport_service_name);
        while (!client->wait_for_service(std::chrono::seconds(1))) 
        {
            const std::string message{"Turtle teleport waiting for service on: " + teleport_service_name};
            RCLCPP_INFO(get_logger(), message.c_str());
        }

        auto future = client->async_send_request(request);
    }

    void killTurtle(const std::string& turtle_name) {
        using Kill = turtlesim::srv::Kill;
        auto request = std::make_shared<Kill::Request>();
        request->name = turtle_name;

        auto client = create_client<Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_INFO(get_logger(), "Turtle teleport waiting for service");
        }

        auto future = client->async_send_request(request);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}