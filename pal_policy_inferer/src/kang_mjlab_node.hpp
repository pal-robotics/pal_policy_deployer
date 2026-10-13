// pal_policy_inferer_mjlab_node (ROS 2)
//
// Runs one or more ONNX RL policies on Kangaroo (mjlab-trained models).
// Configuration is read from ONNX custom metadata; no separate YAML config is needed.
//
// Task types
//   loco      – locomotion policy driven by /cmd_vel velocity commands
//   imitation – autoregressive motion policy; initialises from robot joint state and
//               auto-switches back to policy [0] after a configurable number of frames
//
// Inference is triggered by /introspection_data/values (PAL statistics). All policies
// receive every sensor update so that switching is seamless (no stale observations).
//
// Subscribed topics
//   /joint_states                      sensor_msgs/JointState  (for imitation init only)
//   /introspection_data/names          pal_statistics_msgs/StatisticsNames
//   /introspection_data/values         pal_statistics_msgs/StatisticsValues  (triggers inference)
//   /cmd_vel                           geometry_msgs/Twist  (loco policies only;
//                                   if no message arrives for >1 s the command term falls
//                                   back to zero twist until commands resume)
//   /odom                              nav_msgs/Odometry    (only if base_lin_vel term is used)
//
// Published topics
//   /subscriber_controller/desired_state  sensor_msgs/JointState
//   pal_policy_deployer_statistics        PAL statistics (elapsed time per observation term;
//                                   also carries the raw received cmd_vel.{linear_x,linear_y,
//                                   angular_z} for non-imitation policies)
//
// Action server
//   ~/switch_policy  pal_policy_inferer/action/SwitchPolicy
//
// Impedance gains
//   At startup, on every switch_policy request and on the imitation auto-switch-back, the
//   active policy's ONNX joint_stiffness/joint_damping are written to
//   /rl_impedance_controller gains.<joint>.p / gains.<joint>.d (old -> new values are logged).
//   Requests are queued until the controller's parameter service is available.
//
// Parameters
//   policies         string[]  list of policy names; each name namespaces its own params
//   <name>.task      string    "loco" or "imitation"
//   <name>.model     string    model identifier; selects models/<task>/<task>_<model>_mjlab/
//   <name>.frames    int       (imitation only) auto-return frame budget; 0 = use ONNX default
//   <name>.gains     string    impedance gains folder name (currently unused; reserved)
//
// The node is split over several translation units (kang_mjlab_node*.cpp) to keep the peak
// compiler memory of each one below the 2 GB limit of the debian build.

#ifndef KANG_MJLAB_NODE_HPP_
#define KANG_MJLAB_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>

#include <pal_statistics_msgs/msg/statistics_names.hpp>
#include <pal_statistics_msgs/msg/statistics_values.hpp>

#include <pal_policy_inferer/action/switch_policy.hpp>
#include "pal_policy_inferer/kang_mjlab_inference.hpp"
#include "pal_policy_inferer/kang_mjlab_policy.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace pal_policy_inferer
{

using SwitchPolicy = pal_policy_inferer::action::SwitchPolicy;
using SwitchPolicyGoalHandle = rclcpp_action::ServerGoalHandle<SwitchPolicy>;

// ---- OBSERVATION TERM REGISTRY ----

struct ObservationTerm
{
  std::string name;
  int dim;
  std::vector<double> current;
  std::deque<std::vector<double>> history;
  rclcpp::Time last_msg_time;
  double elapsed_time = 0.0;  // Since sensor update

  // stats_prefix disambiguates multiple policies registering the same term name
  ObservationTerm(
    rclcpp::Node * node,
    const std::string & n, int d, const std::string & stats_prefix = "");

  void initHistory(int history_length)
  {
    history.clear();
    for (int i = 0; i < history_length; ++i) {
      history.push_back(std::vector<double>(dim, 0.0));
    }
  }

  void updateHistory()
  {
    history.pop_front();
    history.push_back(current);
  }
};

// ---- POLICY INSTANCE ----
// All state for one loaded policy, including its ONNX session and observation terms.

struct PolicyInstance
{
  int index = 0;
  std::string name;

  PolicyConfig config;
  MotionState motion;

  std::map<std::string, std::shared_ptr<ObservationTerm>> obs_terms;
  std::vector<std::string> obs_term_order;

  KangMjlabModel model;

  std::vector<double> action;
  std::vector<double> zero_action;
};

// ---- MAIN NODE ----

class KangMjlabNode : public rclcpp::Node
{
public:
  KangMjlabNode()
  : rclcpp::Node("pal_policy_deployer")
  {
  }

  // Two-phase init so we can use shared_from_this() for the action server.
  bool init();

private:
  bool initializeMotionFromRobot(PolicyInstance & p);
  bool initializeObservationTerms(PolicyInstance & p);
  bool loadPolicyConfig(const std::string & model_path, PolicyInstance & p);
  std::vector<float> buildObservationVector(PolicyInstance & p);
  void updateAllHistory(PolicyInstance & p);
  void runInference();

  // Subscriptions / publisher (kang_mjlab_node_ros.cpp)
  void createRosInterfaces();
  void createOdomSubscription();
  void createStatisticsSubscriptions();  // kang_mjlab_node_stats.cpp

  // Impedance gains (kang_mjlab_node_impedance.cpp)
  void createImpedanceGainsInterfaces();
  void impedanceTimerCb();
  /// Read the current gains (for the change log), then write policy `idx`'s metadata gains.
  void applyImpedanceGains(int idx);
  /// Queue the gains of policy `idx` for /rl_impedance_controller. Only the latest request is
  /// kept, so rapid switches never apply a stale policy's gains after a newer one.
  void requestImpedanceGains(int idx) {pending_gains_idx_.store(idx);}

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SwitchPolicy::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle);
  void executeSwitch(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle);

  void jsCB(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void velCmdCB(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void palStatisticsNamesCb(const pal_statistics_msgs::msg::StatisticsNames::ConstSharedPtr msg);
  void palStatisticsValuesCb(const pal_statistics_msgs::msg::StatisticsValues::ConstSharedPtr msg);

  bool loadPolicy(PolicyInstance & p, const std::string & task, const std::string & model_name);

  // ---- MEMBERS ----

  std::vector<PolicyInstance> policies_;
  std::atomic<int> active_policy_idx_{0};

  // Shared robot state (physical robot, independent of policy)
  Eigen::Quaterniond imu_orientation_{1.0, 0.0, 0.0, 0.0};
  bool imu_received_{false};
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_received_{false};

  // cmd_vel (joystick) watchdog: feed zero twist if no command arrives within the timeout.
  static constexpr double cmd_vel_timeout_sec_ = 1.0;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  bool cmd_vel_received_{false};   // true after the first /cmd_vel message
  bool cmd_vel_timed_out_{false};  // edge-tracking flag so we log each transition once

  uint32_t names_version_{0};
  std::vector<std::string> pal_statistics_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr sub_stat_names_;
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr sub_stat_values_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr pub_sub_cmd_;

  rclcpp_action::Server<SwitchPolicy>::SharedPtr switch_action_server_;

  rclcpp::AsyncParametersClient::SharedPtr impedance_params_client_;
  rclcpp::TimerBase::SharedPtr impedance_timer_;
  std::atomic<int> pending_gains_idx_{-1};  // policy whose gains still need sending, -1 = none
};

}  // namespace pal_policy_inferer

#endif  // KANG_MJLAB_NODE_HPP_
