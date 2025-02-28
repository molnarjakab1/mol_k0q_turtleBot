#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <random>

class RandomNumberPublisher : public rclcpp::Node {
public:
  RandomNumberPublisher() : Node("random_number_publisher"), gen(rd()) {

    this->declare_parameter<int>("min_value", 1);
    this->declare_parameter<int>("max_value", 100);
    this->declare_parameter<double>("publish_frequency", 1.0);

    int min_val = this->get_parameter("min_value").as_int();
    int max_val = this->get_parameter("max_value").as_int();
    double freq = this->get_parameter("publish_frequency").as_double();

    dist = std::uniform_int_distribution<int>(min_val, max_val);

    publisher_ = this->create_publisher<std_msgs::msg::Int32>("random_number", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&RandomNumberPublisher::publish_message, this)
    );
  }

private:
  void publish_message() {
    auto message = std_msgs::msg::Int32();
    message.data = dist(gen);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<int> dist;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomNumberPublisher>());
  rclcpp::shutdown();
  return 0;
}
