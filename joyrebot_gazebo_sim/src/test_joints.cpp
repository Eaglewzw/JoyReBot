/**
 * @file test_joints.cpp
 * @brief Closed-loop acceptance test for every simulated B601-RS joint.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

struct JointConfig
{
  std::string joint_name;
  std::string cmd_topic;
  double test_delta;
  double lower;
  double upper;
  double target_tolerance;
  double velocity_tolerance;
  std::string unit;
  std::string label;
};

static const std::vector<JointConfig> JOINTS = {
  {"joint1", "/rebot/joint1/cmd_pos", 0.3, -2.8, 2.8, 0.04, 0.04, "rad", ""},
  {"joint2", "/rebot/joint2/cmd_pos", 0.3, 0.0, 3.14, 0.12, 0.04, "rad", ""},
  {"joint3", "/rebot/joint3/cmd_pos", 0.3, -0.01, 3.14, 0.08, 0.04, "rad", ""},
  {"joint4", "/rebot/joint4/cmd_pos", 0.3, -1.57, 1.57, 0.10, 0.05, "rad", ""},
  {"joint5", "/rebot/joint5/cmd_pos", 0.3, -1.57, 1.57, 0.08, 0.05, "rad", ""},
  {"joint6", "/rebot/joint6/cmd_pos", 0.3, -3.14, 3.14, 0.10, 0.05, "rad", ""},
  {"joint_left", "/rebot/gripper/cmd_pos", 0.015, 0.0, 0.05, 0.004, 0.01, "m", "gripper"},
};

enum class Phase
{
  RESET,
  WAIT_READY,
  START_TEST,
  TRACK_TARGET,
  START_RETURN,
  TRACK_RETURN,
  DONE,
};

struct TestResult
{
  std::string name;
  bool pass;
  double start;
  double target;
  double actual;
  double error;
  std::string unit;
  std::string reason;
};

static std::string join(const std::vector<std::string> & values, const std::string & separator)
{
  std::ostringstream stream;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << separator;
    }
    stream << values[i];
  }
  return stream.str();
}

class JointTester : public rclcpp::Node
{
public:
  JointTester()
  : Node("test_joints")
  {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_state_callback(message);
      });

    for (const auto & config : JOINTS) {
      publishers_[config.joint_name] =
        create_publisher<std_msgs::msg::Float64>(config.cmd_topic, 10);
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() { timer_callback(); });
    RCLCPP_INFO(get_logger(), "JointTester started. Resetting all joints to home ...");
  }

  bool succeeded() const {return finished_ && all_passed_;}

private:
  static constexpr int kResetTimeoutTicks = 150;  // 15 s
  static constexpr int kReadyTimeoutTicks = 100;   // 10 s
  static constexpr int kTargetTimeoutTicks = 70;   // 7 s
  static constexpr int kReturnTimeoutTicks = 80;   // 8 s
  static constexpr int kStableSamples = 3;
  static constexpr double kResetTolerance = 0.08;   // rad/m for reset settle check

  void joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr message)
  {
    const size_t count = std::min(message->name.size(), message->position.size());
    if (message->name.size() != message->position.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Malformed /joint_states: name=%zu, position=%zu",
        message->name.size(), message->position.size());
    }

    for (size_t i = 0; i < count; ++i) {
      positions_[message->name[i]] = message->position[i];
      velocities_[message->name[i]] =
        i < message->velocity.size() ? message->velocity[i] : 0.0;
    }
    ++state_sequence_;
    last_state_time_ = std::chrono::steady_clock::now();

    if (!received_state_) {
      received_state_ = true;
      RCLCPP_INFO(
        get_logger(), "Received /joint_states. Joints found: [%s]",
        join(message->name, ", ").c_str());
    }
  }

  void timer_callback()
  {
    switch (phase_) {
      case Phase::RESET: wait_reset(); break;
      case Phase::WAIT_READY: wait_until_ready(); break;
      case Phase::START_TEST: start_test(); break;
      case Phase::TRACK_TARGET: track_target(); break;
      case Phase::START_RETURN: start_return(); break;
      case Phase::TRACK_RETURN: track_return(); break;
      case Phase::DONE: break;
    }
  }

  void wait_reset()
  {
    if (++phase_ticks_ > kResetTimeoutTicks) {
      RCLCPP_WARN(get_logger(), "Reset timed out; proceeding anyway");
      phase_ticks_ = 0;
      phase_ = Phase::WAIT_READY;
      return;
    }
    if (!received_state_) { return; }

    // Command every joint to its expected home position.
    for (const auto & config : JOINTS) {
      std_msgs::msg::Float64 msg;
      msg.data = 0.0;  // home position for all joints/gripper
      publishers_[config.joint_name]->publish(msg);
    }

    // Check whether all test joints are within tolerance of zero.
    bool all_home = true;
    for (const auto & config : JOINTS) {
      auto it = positions_.find(config.joint_name);
      if (it == positions_.end()) { all_home = false; break; }
      if (std::abs(it->second) > kResetTolerance) { all_home = false; break; }
    }
    if (all_home) {
      RCLCPP_INFO(get_logger(), "All joints at home position. Proceeding to test.");
      phase_ticks_ = 0;
      phase_ = Phase::WAIT_READY;
    }
  }

  void wait_until_ready()
  {
    if (++phase_ticks_ > kReadyTimeoutTicks) {
      finish_with_environment_error("Timed out waiting for /joint_states and command bridges");
      return;
    }
    if (!received_state_) {
      return;
    }

    std::vector<std::string> missing_joints;
    std::vector<std::string> disconnected_topics;
    for (const auto & config : JOINTS) {
      if (positions_.count(config.joint_name) == 0) {
        missing_joints.push_back(config.joint_name);
      }
      if (publishers_[config.joint_name]->get_subscription_count() == 0) {
        disconnected_topics.push_back(config.cmd_topic);
      }
    }
    if (!missing_joints.empty() || !disconnected_topics.empty()) {
      return;
    }

    const size_t state_publishers = joint_state_sub_->get_publisher_count();
    if (state_publishers != 1) {
      finish_with_environment_error(
        "Expected exactly one /joint_states publisher, found " +
        std::to_string(state_publishers) + ". Stop duplicate simulations.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Simulation and all command bridges are ready.");
    phase_ticks_ = 0;
    phase_ = Phase::START_TEST;
  }

  void start_test()
  {
    const auto & config = JOINTS[joint_index_];
    const double current = positions_.at(config.joint_name);
    start_position_ = current;

    if (current + config.test_delta <= config.upper) {
      target_position_ = current + config.test_delta;
    } else if (current - config.test_delta >= config.lower) {
      target_position_ = current - config.test_delta;
    } else {
      target_position_ = std::clamp(current, config.lower, config.upper);
    }

    command_state_sequence_ = state_sequence_;
    phase_ticks_ = 0;
    stable_samples_ = 0;
    publish_target(target_position_);
    RCLCPP_INFO(
      get_logger(), "[%s] target=%.4f %s, start=%.4f, tolerance=%.4f",
      label(config).c_str(), target_position_, config.unit.c_str(), current,
      config.target_tolerance);
    phase_ = Phase::TRACK_TARGET;
  }

  void track_target()
  {
    const auto & config = JOINTS[joint_index_];
    ++phase_ticks_;
    publish_target(target_position_);  // tolerate bridge discovery / packet loss

    if (!state_is_fresh() || state_sequence_ <= command_state_sequence_) {
      if (phase_ticks_ >= kTargetTimeoutTicks) {
        record_result(false, "no fresh joint state after command");
        phase_ = Phase::START_RETURN;
      }
      return;
    }

    const double actual = positions_.at(config.joint_name);
    const double error = std::abs(actual - target_position_);
    const double velocity = std::abs(velocities_[config.joint_name]);
    stable_samples_ =
      error <= config.target_tolerance && velocity <= config.velocity_tolerance ?
      stable_samples_ + 1 : 0;

    if (stable_samples_ >= kStableSamples) {
      record_result(true, "target reached and stable");
      phase_ = Phase::START_RETURN;
    } else if (phase_ticks_ >= kTargetTimeoutTicks) {
      const bool correct_direction =
        (actual - start_position_) * (target_position_ - start_position_) > 0.0;
      record_result(
        false, correct_direction ? "target tolerance not reached" : "no response or wrong direction");
      phase_ = Phase::START_RETURN;
    }
  }

  void start_return()
  {
    command_state_sequence_ = state_sequence_;
    phase_ticks_ = 0;
    stable_samples_ = 0;
    publish_target(start_position_);
    phase_ = Phase::TRACK_RETURN;
  }

  void track_return()
  {
    const auto & config = JOINTS[joint_index_];
    ++phase_ticks_;
    publish_target(start_position_);

    if (state_is_fresh() && state_sequence_ > command_state_sequence_) {
      const double error = std::abs(positions_.at(config.joint_name) - start_position_);
      const double velocity = std::abs(velocities_[config.joint_name]);
      stable_samples_ =
        error <= config.target_tolerance && velocity <= config.velocity_tolerance ?
        stable_samples_ + 1 : 0;
      if (stable_samples_ >= kStableSamples) {
        advance();
        return;
      }
    }

    if (phase_ticks_ >= kReturnTimeoutTicks) {
      RCLCPP_WARN(
        get_logger(), "[%s] did not fully return before timeout; continuing",
        label(config).c_str());
      advance();
    }
  }

  void publish_target(double target)
  {
    std_msgs::msg::Float64 message;
    message.data = target;
    publishers_[JOINTS[joint_index_].joint_name]->publish(message);
  }

  bool state_is_fresh() const
  {
    return received_state_ &&
           std::chrono::steady_clock::now() - last_state_time_ < std::chrono::seconds(1);
  }

  void record_result(bool pass, const std::string & reason)
  {
    const auto & config = JOINTS[joint_index_];
    const double actual = positions_.at(config.joint_name);
    const double error = std::abs(actual - target_position_);
    results_.push_back(
      {label(config), pass, start_position_, target_position_, actual, error,
        config.unit, reason});
    RCLCPP_INFO(
      get_logger(), "[%s] %s start=%.4f target=%.4f actual=%.4f error=%.4f (%s)",
      label(config).c_str(), pass ? "PASS" : "FAIL", start_position_, target_position_,
      actual, error, reason.c_str());
  }

  void advance()
  {
    ++joint_index_;
    if (joint_index_ >= JOINTS.size()) {
      print_report();
      finished_ = true;
      all_passed_ = std::all_of(
        results_.begin(), results_.end(), [](const TestResult & result) {return result.pass;});
      phase_ = Phase::DONE;
      rclcpp::shutdown();
      return;
    }
    phase_ = Phase::START_TEST;
  }

  void finish_with_environment_error(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "Environment error: %s", reason.c_str());
    finished_ = true;
    all_passed_ = false;
    phase_ = Phase::DONE;
    rclcpp::shutdown();
  }

  void print_report() const
  {
    std::ostringstream report;
    report << "\n============================================================\n";
    report << "  JoyReBot Closed-loop Joint Test Report\n";
    report << "============================================================\n";
    size_t passed = 0;
    for (const auto & result : results_) {
      passed += result.pass ? 1 : 0;
      report << "  " << std::left << std::setw(10) << result.name
             << (result.pass ? "PASS  " : "FAIL  ")
             << std::fixed << std::setprecision(4)
             << "target=" << result.target << ", actual=" << result.actual
             << ", error=" << result.error << " " << result.unit
             << "  [" << result.reason << "]\n";
    }
    report << "------------------------------------------------------------\n";
    report << "  Result: " << passed << "/" << results_.size() << " targets reached\n";
    report << "============================================================\n";
    RCLCPP_INFO(get_logger(), "%s", report.str().c_str());
  }

  static std::string label(const JointConfig & config)
  {
    return config.label.empty() ? config.joint_name : config.label;
  }

  Phase phase_{Phase::RESET};
  size_t joint_index_{0};
  int phase_ticks_{0};
  int stable_samples_{0};
  bool received_state_{false};
  bool finished_{false};
  bool all_passed_{false};
  uint64_t state_sequence_{0};
  uint64_t command_state_sequence_{0};
  double start_position_{0.0};
  double target_position_{0.0};
  std::chrono::steady_clock::time_point last_state_time_{};

  std::map<std::string, double> positions_;
  std::map<std::string, double> velocities_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
  std::vector<TestResult> results_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTester>();
  rclcpp::spin(node);
  return node->succeeded() ? 0 : 1;
}
