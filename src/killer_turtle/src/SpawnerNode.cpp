#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

#include "turtle_msgs/msg/turtle.hpp"
#include "../include/killer_turtle/Position.hpp"

Position random_spawn_position() {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 10.0);

    Position spawn;
    spawn.x = dist(mt);
    spawn.y = dist(mt);

    return spawn;
}

class SpawnerNode : public rclcpp::Node
{
using Turtle = turtle_msgs::msg::Turtle;
public:
    SpawnerNode() : Node("spawner_node")
    {
        declare_parameter("spawn_cooldown", 1);
        const int spawn_cooldown{get_parameter("spawn_cooldown").as_int()};

        new_turtle_publisher_ = create_publisher<Turtle>("new_turtle", 10);

        timer_ = create_wall_timer(std::chrono::seconds(spawn_cooldown), [this]() {
            const Turtle new_turtle {createNewTurtle()};
            spawnTurtle(new_turtle);
            new_turtle_publisher_->publish(new_turtle);
            turtle_counter_++;
        });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Turtle>::SharedPtr new_turtle_publisher_;
    int turtle_counter_ {0};

    void spawnTurtle(const Turtle& new_turtle) {
        using Spawn = turtlesim::srv::Spawn;

        auto client = create_client<Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_INFO(get_logger(), "Turtle spawn waiting for service");
        }

        auto request = std::make_shared<Spawn::Request>();
        request->x = new_turtle.pose.x;
        request->y = new_turtle.pose.y;
        request->name = new_turtle.name;

        auto future = client->async_send_request(request);
    }

    Turtle createNewTurtle() const {
        Turtle new_turtle;
        new_turtle.id = turtle_counter_;
        new_turtle.name = "spawned_turtle_" + std::to_string(turtle_counter_);

        const Position spawn {random_spawn_position()};
        new_turtle.pose.x = spawn.x;
        new_turtle.pose.y = spawn.y;

        return new_turtle;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}