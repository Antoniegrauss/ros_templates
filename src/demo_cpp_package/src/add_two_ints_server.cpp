#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsServerNode : public rclcpp::Node
{
    using AddInts = example_interfaces::srv::AddTwoInts;
    using AddService = rclcpp::Service<AddInts>;

public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = create_service<AddInts>("add_two_ints", [this](
            
            const AddInts::Request::SharedPtr request, 
            const AddInts::Response::SharedPtr response) 
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(get_logger(), "%d + %d = %d", request->a, request->b, response->sum);
        });

        RCLCPP_INFO(get_logger(), "Service server has started");
    }

private:
    AddService::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}