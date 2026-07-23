#ifndef JOINTS_SELF_CHECK_HPP_
#define JOINTS_SELF_CHECK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ── Configuration & data types ─────────────────────────────────────────────────

struct JointConfig
{
  std::string joint_name;       ///< /joint_states 及 URDF 中的关节名
  std::string cmd_topic;        ///< 位置指令话题
  double test_delta;            ///< 测试偏移量（从当前位置 ± 该值）
  double lower;                 ///< 关节下限 — 取自 URDF <limit lower="…">
  double upper;                 ///< 关节上限 — 取自 URDF <limit upper="…">
  double target_tolerance;      ///< 到达目标的最大位置误差
  double velocity_tolerance;    ///< 判定稳定的最大速度
  std::string unit;             ///< "rad" = 旋转关节, "m" = 直线关节
  std::string label;            ///< 测试报告中的显示名（空则用 joint_name）
};

/// Ordered list of joints tested by the node.
extern const std::vector<JointConfig> JOINTS;

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

/// Join a vector of strings with a separator.
std::string join(const std::vector<std::string> & values, const std::string & separator);

// ── JointTester node ───────────────────────────────────────────────────────────

class JointTester : public rclcpp::Node
{
public:
  JointTester();
  bool succeeded() const;

private:
  static constexpr int kResetTimeoutTicks = 150;   // 15 s
  static constexpr int kReadyTimeoutTicks = 100;   // 10 s
  static constexpr int kTargetTimeoutTicks = 70;   // 7 s
  static constexpr int kReturnTimeoutTicks = 80;   // 8 s
  static constexpr int kStableSamples = 3;
  static constexpr double kResetTolerance = 0.08;  // rad/m for reset settle check

  void joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr message);
  void timer_callback();

  // Phase handlers
  void wait_reset();
  void wait_until_ready();
  void start_test();
  void track_target();
  void start_return();
  void track_return();

  // Helpers
  void publish_target(double target);
  bool state_is_fresh() const;
  void record_result(bool pass, const std::string & reason);
  void advance();
  void finish_with_environment_error(const std::string & reason);
  void print_report() const;
  static std::string label(const JointConfig & config);

  // ── State ──────────────────────────────────────────────────────────────────
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

#endif  // JOYREBOT_GAZEBO_SIM__TEST_JOINTS_HPP_
