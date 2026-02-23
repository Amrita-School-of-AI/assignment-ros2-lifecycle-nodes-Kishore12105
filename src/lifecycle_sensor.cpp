#include <memory>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleSensor()
  : rclcpp_lifecycle::LifecycleNode("lifecycle_sensor"),
    generator_(std::random_device{}()),
    distribution_(0.0, 100.0)
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Sensor node created");
  }

private:
  // Publisher
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Random number generator
  std::mt19937 generator_;
  std::uniform_real_distribution<double> distribution_;

  // ================= Lifecycle Callbacks =================

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensor_data", 10);

    RCLCPP_INFO(get_logger(), "Sensor configured");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();

    timer_ = this->create_wall_timer(
      500ms,
      [this]() {
        auto msg = std_msgs::msg::Float64();
        msg.data = distribution_(generator_);

        publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), "Publishing sensor data: %.2f", msg.data);
      });

    RCLCPP_INFO(get_logger(), "Sensor activated");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (timer_) {
      timer_->cancel();
    }

    publisher_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Sensor deactivated");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher_.reset();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Sensor cleaned up");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Sensor shutting down");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleSensor>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}