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
  : LifecycleNode("lifecycle_sensor")
  {
    // ── IMPORTANT ──
    // Do NOT create publishers / timers / subscriptions here in most lifecycle nodes
  }

private:
  // ───────────────────────────────────────────────
  //  on_configure
  // ───────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/sensor_data", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Sensor configured");

    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────
  //  on_activate
  // ───────────────────────────────────────────────
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    using namespace std::placeholders;

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&LifecycleSensor::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Sensor activated");

    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────
  //  on_deactivate
  // ───────────────────────────────────────────────
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    timer_.reset();

    RCLCPP_INFO(this->get_logger(), "Sensor deactivated");

    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────
  //  on_cleanup
  // ───────────────────────────────────────────────
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher_.reset();
    timer_.reset();   // usually already reset, but safe

    RCLCPP_INFO(this->get_logger(), "Sensor cleaned up");

    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────
  //  on_shutdown    (called from any state on shutdown)
  // ───────────────────────────────────────────────
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(),
      "Sensor shutting down from state: %s", state.label().c_str());

    publisher_.reset();
    timer_.reset();

    return CallbackReturn::SUCCESS;
  }

  // ───────────────────────────────────────────────
  //  Timer callback ─ only publishing when ACTIVE
  // ───────────────────────────────────────────────
  void on_timer()
  {
    if (!publisher_ || !this->publisher_->is_activated()) {
      return;
    }

    auto msg = std_msgs::msg::Float64();
    msg.data = dist_(gen_);

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Publishing sensor data: %.2f", msg.data);
  }

  // ── Member variables ────────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Random number generator (0.0 – 100.0)
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> dist{0.0, 100.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleSensor>();

  // Very important: use the lifecycle spin interface
  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}