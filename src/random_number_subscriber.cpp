#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class RandomNumberSubscriber : public rclcpp::Node
{
public:
    RandomNumberSubscriber() : Node("random_number_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "random_number", 10, 
            std::bind(&RandomNumberSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %d", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomNumberSubscriber>());
    rclcpp::shutdown();
    return 0;
}
