#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <memory>
#include <random>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleSensor()
  : LifecycleNode("lifecycle_sensor"),
    publisher_(nullptr),
    timer_(nullptr)
  {
    // Nothing here — important: do not create pub/timer in constructor
  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensor_data", 10);

    RCLCPP_INFO(this->get_logger(), "Sensor configured");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&LifecycleSensor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Sensor activated");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    timer_.reset();

    RCLCPP_INFO(this->get_logger(), "Sensor deactivated");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher_.reset();
    timer_.reset();

    RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "Sensor shutting down (from state: %s)",
                state.label().c_str());

    publisher_.reset();
    timer_.reset();

    return CallbackReturn::SUCCESS;
  }

  void timer_callback()
  {
    if (!publisher_ || !publisher_->is_activated()) {
      return;
    }

    auto msg = std_msgs::msg::Float64();
    msg.data = dist_(gen_);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
  }

  // ── Members ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_real_distribution<double> dist_{0.0, 100.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleSensor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}