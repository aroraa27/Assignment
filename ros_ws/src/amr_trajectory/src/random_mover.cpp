#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class RandomMover : public rclcpp::Node
{
public:
  RandomMover()
  : Node("random_mover")
  {

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Timer to publish random twist messages at a 500ms interval.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RandomMover::publish_random_twist, this));

    // Initialize random number generator.
    std::random_device rd;
    generator_ = std::mt19937(rd());

    // Linear velocity between 0 and 1 m/s
    linear_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    // Angular velocity between -1 and 1 rad/s
    angular_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);
  }

private:
  // Publisher to the turtlesim
  void publish_random_twist()
  {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_dist_(generator_);
    twist_msg.angular.z = angular_dist_(generator_);
    RCLCPP_INFO(this->get_logger(), "Publishing Twist: linear=%.2f, angular=%.2f",
                twist_msg.linear.x, twist_msg.angular.z);
    publisher_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 generator_;
  std::uniform_real_distribution<double> linear_dist_;
  std::uniform_real_distribution<double> angular_dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomMover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
