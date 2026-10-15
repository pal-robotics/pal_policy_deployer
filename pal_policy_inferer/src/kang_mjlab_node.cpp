#include "kang_mjlab_node.hpp"

#include <ament_index_cpp/version.h>
// get_package_share_directory() was replaced by get_package_share_path() in
// ament_index_cpp >= 1.13.2 (https://github.com/ament/ament_index/pull/120).
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 2)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <pal_statistics/pal_statistics_macros.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

static const std::string POLICY_DEPLOYER_TOPIC = "pal_policy_deployer_statistics";

namespace pal_policy_inferer
{

ObservationTerm::ObservationTerm(
  rclcpp::Node * node,
  const std::string & n, int d, const std::string & stats_prefix)
: name(n), dim(d), current(d, 0.0), last_msg_time(0, 0, RCL_ROS_TIME), elapsed_time(0.0)
{
  const std::string full_name = "elapsed_time." + stats_prefix + name;
  REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, full_name, &elapsed_time);
  if(name.compare("imu_projected_gravity") == 0 && current.size() == 3)
  {
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "imu_projected_gravity.x", &current[0]);
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "imu_projected_gravity.y", &current[1]);
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "imu_projected_gravity.z", &current[2]);
  }
  if(name.compare("command") == 0 && current.size() == 3)
  {
    // dim==3 for the "command" term only occurs for non-imitation policies:
    // getObservationTermDim() returns mot.commandDim() (= num_joints*2, always even)
    // for imitation, and 3 otherwise.
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "cmd_vel.linear_x", &current[0]);
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "cmd_vel.linear_y", &current[1]);
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, "cmd_vel.angular_z", &current[2]);
  }
}

bool KangMjlabNode::init()
{
  struct PolicySpec
  {
    std::string task, model, name, gains;
    int frames = 0;
  };
  std::vector<PolicySpec> specs;

  this->declare_parameter<std::vector<std::string>>("policies", std::vector<std::string>{});
  const auto names = this->get_parameter("policies").as_string_array();
  if (names.empty()) {
    RCLCPP_FATAL(
      get_logger(),
      "No policies configured. Set the 'policies' parameter to a non-empty string array.");
    return false;
  }

  for (const auto & name : names) {
    this->declare_parameter<std::string>(name + ".task", "");
    this->declare_parameter<std::string>(name + ".model", "");
    this->declare_parameter<int>(name + ".frames", 0);
    this->declare_parameter<std::string>(name + ".gains", "impedance_control");

    PolicySpec s;
    s.name = name;
    s.task = this->get_parameter(name + ".task").as_string();
    s.model = this->get_parameter(name + ".model").as_string();
    s.frames = static_cast<int>(this->get_parameter(name + ".frames").as_int());
    s.gains = this->get_parameter(name + ".gains").as_string();
    if (s.task.empty() || s.model.empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "Policy '%s' is missing required parameter '%s.task' or '%s.model'.",
        name.c_str(), name.c_str(), name.c_str());
      return false;
    }
    specs.push_back(s);
  }

  // Subscribers / publisher created before policy loading so topics are always visible.
  createRosInterfaces();
  createStatisticsSubscriptions();

  switch_action_server_ = rclcpp_action::create_server<SwitchPolicy>(
    this, "~/switch_policy",
    std::bind(
      &KangMjlabNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&KangMjlabNode::handleCancel, this, std::placeholders::_1),
    std::bind(&KangMjlabNode::handleAccepted, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(),
    "Action server '~/switch_policy' started (pal_policy_inferer/action/SwitchPolicy)");

  RCLCPP_INFO(
    get_logger(), "Loading %zu polic%s...",
    specs.size(), specs.size() == 1 ? "y" : "ies");

  policies_.resize(specs.size());
  for (size_t i = 0; i < specs.size(); ++i) {
    policies_[i].index = static_cast<int>(i);
    policies_[i].name = specs[i].name;
    if (!loadPolicy(policies_[i], specs[i].task, specs[i].model)) {
      RCLCPP_FATAL(
        get_logger(), "Failed to load policy [%zu] '%s'",
        i, specs[i].name.c_str());
      return false;
    }
    if (specs[i].frames > 0) {
      policies_[i].motion.frames = specs[i].frames;
    }
    // TODO impedance: build impedance request from specs[i].gains (deferred).
  }

  RCLCPP_INFO(
    get_logger(), "=== %zu polic%s loaded ===",
    policies_.size(), policies_.size() == 1 ? "y" : "ies");
  for (size_t i = 0; i < policies_.size(); ++i) {
    RCLCPP_INFO(
      get_logger(), "  [%zu] '%s' (task=%s)%s",
      i, policies_[i].name.c_str(),
      policies_[i].config.task.c_str(), i == 0 ? " <-- active" : "");
  }

  bool need_odom = false;
  for (auto & p : policies_) {
    if (p.obs_terms.find("base_lin_vel") != p.obs_terms.end()) {
      need_odom = true;
      break;
    }
  }
  if (need_odom) {
    createOdomSubscription();
    RCLCPP_INFO(get_logger(), "Subscribed to /odom for base_lin_vel");
  }

  createImpedanceGainsInterfaces();
  requestImpedanceGains(0);

  RCLCPP_INFO(
    get_logger(), "Policy deployer started. Active policy: [0] '%s'",
    policies_[0].name.c_str());
  if (policies_[0].config.task == "imitation") {
    RCLCPP_INFO(get_logger(), "Waiting for joint state to initialize motion...");
  }

  return true;
}

// ---- INITIALIZE MOTION FROM ROBOT STATE ----

bool KangMjlabNode::initializeMotionFromRobot(PolicyInstance & p)
{
  if (!joint_state_received_) {
    RCLCPP_WARN(
      get_logger(),
      "Waiting for joint state to initialize motion for policy '%s'...", p.name.c_str());
    return false;
  }

  p.motion.num_joints = static_cast<int>(p.config.joint_names.size());
  p.motion.joint_pos.resize(p.motion.num_joints, 0.0);
  p.motion.joint_vel.resize(p.motion.num_joints, 0.0);

  for (int j = 0; j < p.motion.num_joints; ++j) {
    const std::string & joint_name = p.config.joint_names[j];
    for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
      if (latest_joint_state_.name[i] == joint_name) {
        p.motion.joint_pos[j] = latest_joint_state_.position[i];
        if (i < latest_joint_state_.velocity.size()) {
          p.motion.joint_vel[j] = latest_joint_state_.velocity[i];
        }
        break;
      }
    }
  }

  if (p.motion.frames == 0) {
    // Match the ROS 1 fallback default if no per-policy frames were configured.
    p.motion.frames = 6000;
  }
  p.motion.timestep = 0;
  p.motion.loaded = true;

  RCLCPP_INFO(
    get_logger(), "Motion initialized for policy '%s': %d joints, frames=%d",
    p.name.c_str(), p.motion.num_joints, p.motion.frames);
  return true;
}

// ---- OBSERVATION TERM INITIALIZATION ----

bool KangMjlabNode::initializeObservationTerms(PolicyInstance & p)
{
  p.obs_terms.clear();
  p.obs_term_order.clear();
  const std::string prefix = "p" + std::to_string(p.index) + ".";

  for (const auto & term_name : p.config.observation_names) {
    int dim = getObservationTermDim(p.config, p.motion, term_name);
    if (dim < 0) {
      RCLCPP_FATAL(
        get_logger(),
        "[Policy '%s'] Unknown observation term: '%s'. Add it to getObservationTermDim().",
        p.name.c_str(), term_name.c_str());
      return false;
    }
    auto term = std::make_shared<ObservationTerm>(this, term_name, dim, prefix);
    term->initHistory(p.config.history_length);
    p.obs_terms[term_name] = term;
    p.obs_term_order.push_back(term_name);
    RCLCPP_INFO(
      get_logger(), "  [%s] Term '%s': dim=%d",
      p.name.c_str(), term_name.c_str(), dim);
  }
  return true;
}

// ---- ONNX MODEL LOADING ----

bool KangMjlabNode::loadPolicyConfig(const std::string & model_path, PolicyInstance & p)
{
  // Only catch our own config errors (missing metadata key, obs/history mismatch);
  // malformed numbers and Ort::Exception propagate as before.
  try {
    for (const auto & line : p.model.load(model_path, p.config, p.motion)) {
      RCLCPP_INFO(get_logger(), "  %s", line.c_str());
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(get_logger(), "%s", e.what());
    return false;
  }

  if (p.config.action_dim != static_cast<int>(p.config.action_joint_names.size())) {
    RCLCPP_WARN(
      get_logger(), "Action dim mismatch: ONNX=%d, expected=%zu",
      p.config.action_dim, p.config.action_joint_names.size());
  }

  return true;
}

// ---- BUILD OBSERVATION VECTOR ----

std::vector<float> KangMjlabNode::buildObservationVector(PolicyInstance & p)
{
  std::vector<float> obs;
  obs.reserve(p.config.obs_dim);
  const auto obs_time = this->now();
  for (const auto & term_name : p.obs_term_order) {
    auto & term = p.obs_terms[term_name];
    term->elapsed_time = (obs_time - term->last_msg_time).seconds();
    for (int t = 0; t < p.config.history_length; ++t) {
      for (int i = 0; i < term->dim; ++i) {
        obs.push_back(static_cast<float>(term->history[t][i]));
      }
    }
  }
  return obs;
}

void KangMjlabNode::updateAllHistory(PolicyInstance & p)
{
  for (auto & pair : p.obs_terms) {
    pair.second->updateHistory();
  }
}

// ---- INFERENCE ----

void KangMjlabNode::runInference()
{
  PolicyInstance & p = policies_[active_policy_idx_.load()];

  if (p.config.task == "imitation" && !p.motion.loaded) {
    if (!initializeMotionFromRobot(p)) {
      return;
    }
  }

  std::vector<float> obs_input;
  float time_step_val = 0.0f;
  {
    if (p.config.task == "imitation" && p.motion.loaded) {
      if (p.obs_terms.find("command") != p.obs_terms.end()) {
        auto & term = p.obs_terms["command"];
        std::vector<double> cmd = p.motion.getCommand();
        for (int i = 0; i < term->dim && i < static_cast<int>(cmd.size()); ++i) {
          term->current[i] = cmd[i];
        }
      }
      time_step_val = static_cast<float>(p.motion.timestep);
    } else if (p.config.task != "imitation" && cmd_vel_received_) {
      // Watchdog: if /cmd_vel has not arrived for more than cmd_vel_timeout_sec_,
      // zero the command term so the policy does not act on a stale velocity command.
      auto it = p.obs_terms.find("command");
      if (it != p.obs_terms.end()) {
        const double since_last = (this->now() - last_cmd_vel_time_).seconds();
        if (since_last > cmd_vel_timeout_sec_) {
          if (!cmd_vel_timed_out_) {
            RCLCPP_WARN(
              get_logger(),
              "No cmd_vel for %.2fs (> %.1fs) sending zero twist to policy '%s'",
              since_last, cmd_vel_timeout_sec_, p.name.c_str());
            cmd_vel_timed_out_ = true;
          }
          std::fill(it->second->current.begin(), it->second->current.end(), 0.0);
        }
      }
    }

    updateAllHistory(p);
    obs_input = buildObservationVector(p);
  }
  PUBLISH_STATISTICS(this, POLICY_DEPLOYER_TOPIC);

  const auto outputs = p.model.run(
    obs_input, time_step_val, p.config.num_inputs > 1 && p.config.task == "imitation");

  const float * actions_data = outputs.actions;
  for (int i = 0; i < p.config.action_dim; ++i) {
    p.action[i] = p.config.action_scale[i] * actions_data[i] + p.config.action_offset[i];
  }

  if (p.obs_terms.find("actions") != p.obs_terms.end()) {
    auto & actions_term = p.obs_terms["actions"];
    actions_term->last_msg_time = this->now();
    for (int i = 0; i < p.config.action_dim; ++i) {
      actions_term->current[i] = actions_data[i];
    }
  }

  if (p.config.task == "imitation" && p.motion.loaded) {
    if (outputs.joint_pos && outputs.joint_vel) {
      p.motion.updateFromOutputs(
        outputs.joint_pos, p.motion.num_joints, outputs.joint_vel, p.motion.num_joints);
    }
    p.motion.timestep++;
    if (p.motion.frames > 0 && p.motion.timestep >= p.motion.frames) {
      RCLCPP_INFO(
        get_logger(),
        "Imitation policy '%s' reached frames=%d, switching back to default policy [0]",
        p.name.c_str(), p.motion.frames);
      p.motion.loaded = false;
      p.motion.timestep = 0;
      active_policy_idx_.store(0);
      requestImpedanceGains(0);
    }
  }

  control_msgs::msg::MultiDOFCommand cmd_msg;
  cmd_msg.dof_names = p.config.action_joint_names;
  cmd_msg.values = p.action;
  cmd_msg.values_dot = p.zero_action;
  pub_sub_cmd_->publish(cmd_msg);
}

// ---- POLICY SWITCH ACTION ----

rclcpp_action::GoalResponse KangMjlabNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const SwitchPolicy::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received switch_policy request: index=%d", goal->policy_index);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse KangMjlabNode::handleCancel(
  const std::shared_ptr<SwitchPolicyGoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void KangMjlabNode::handleAccepted(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle)
{
  std::thread{std::bind(&KangMjlabNode::executeSwitch, this, std::placeholders::_1),
    goal_handle}.detach();
}

void KangMjlabNode::executeSwitch(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle)
{
  auto result = std::make_shared<SwitchPolicy::Result>();
  const auto goal = goal_handle->get_goal();
  int new_idx = goal->policy_index;

  if (new_idx < 0 || new_idx >= static_cast<int>(policies_.size())) {
    result->success = false;
    result->message = "Invalid policy index " + std::to_string(new_idx) +
      " (have " + std::to_string(policies_.size()) + " policies)";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  int cur_idx = active_policy_idx_.load();
  if (policies_[cur_idx].config.task == "imitation" && policies_[cur_idx].motion.loaded) {
    result->success = false;
    result->message = "Cannot switch policy: imitation policy [" + std::to_string(cur_idx) +
      "] '" + policies_[cur_idx].name + "' is still running (timestep=" +
      std::to_string(policies_[cur_idx].motion.timestep) + "/" +
      std::to_string(policies_[cur_idx].motion.frames) + ")";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  int old_idx = active_policy_idx_.exchange(new_idx);

  if (policies_[new_idx].config.task == "imitation") {
    policies_[new_idx].motion.loaded = false;
    policies_[new_idx].motion.timestep = 0;
  }

  RCLCPP_INFO(
    get_logger(), "Switched policy: [%d] '%s' -> [%d] '%s'",
    old_idx, policies_[old_idx].name.c_str(),
    new_idx, policies_[new_idx].name.c_str());

  requestImpedanceGains(new_idx);

  result->success = true;
  result->message = "Switched to policy [" + std::to_string(new_idx) + "] '" +
    policies_[new_idx].name + "'";
  goal_handle->succeed(result);
}

// ---- SENSOR CALLBACKS ----
// Each callback updates ALL policies so that switching is seamless (no stale obs).

void KangMjlabNode::jsCB(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // Keep latest joint state for imitation policy initialisation only.
  latest_joint_state_ = *msg;
  if (!joint_state_received_) {
    joint_state_received_ = true;
  }
}

void KangMjlabNode::velCmdCB(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  last_cmd_vel_time_ = this->now();
  cmd_vel_received_ = true;
  if (cmd_vel_timed_out_) {
    RCLCPP_INFO(get_logger(), "cmd_vel resumed restoring velocity commands to the policy");
    cmd_vel_timed_out_ = false;
  }

  for (auto & p : policies_) {
    if (p.config.task == "imitation") {continue;}
    if (p.obs_terms.find("command") != p.obs_terms.end()) {
      auto & term = p.obs_terms["command"];
      term->last_msg_time = this->now();
      term->current[0] = msg->linear.x;
      term->current[1] = msg->linear.y;
      term->current[2] = msg->angular.z;
    }
  }
}

void KangMjlabNode::odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  for (auto & p : policies_) {
    if (p.obs_terms.find("base_lin_vel") != p.obs_terms.end()) {
      auto & term = p.obs_terms["base_lin_vel"];
      term->last_msg_time = msg->header.stamp;
      term->current[0] = msg->twist.twist.linear.x;
      term->current[1] = msg->twist.twist.linear.y;
      term->current[2] = msg->twist.twist.linear.z;
    }
  }
}

void KangMjlabNode::palStatisticsNamesCb(
  const pal_statistics_msgs::msg::StatisticsNames::ConstSharedPtr msg)
{
  names_version_ = msg->names_version;
  pal_statistics_names_ = msg->names;
}

void KangMjlabNode::palStatisticsValuesCb(
  const pal_statistics_msgs::msg::StatisticsValues::ConstSharedPtr msg)
{
  if (pal_statistics_names_.empty()) {
    RCLCPP_WARN(get_logger(), "Skipping the loop as the names is not received yet!");
    return;
  }
  if (names_version_ != msg->names_version) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping the reading as the pal statistics names and values version is different");
    return;
  }

  auto hasKey = [&](const std::string & k) {
      return std::find(pal_statistics_names_.cbegin(), pal_statistics_names_.cend(), k)
             != pal_statistics_names_.cend();
    };
  if (!hasKey("state_interface.torso_imu_sensor/orientation.w") ||
    !hasKey("state_interface.torso_imu_sensor/angular_velocity.x") ||
    !hasKey("state_interface.torso_imu_sensor/linear_acceleration.x"))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Waiting for IMU statistics — controller not started yet?");
    return;
  }

  const auto quat_w_index = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/orientation.w");
  const auto quat_x_index = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/orientation.x");
  const auto quat_y_index = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/orientation.y");
  const auto quat_z_index = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/orientation.z");

  const auto ang_vel_x_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/angular_velocity.x");
  const auto ang_vel_y_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/angular_velocity.y");
  const auto ang_vel_z_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/angular_velocity.z");

  const auto lin_accel_x_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/linear_acceleration.x");
  const auto lin_accel_y_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/linear_acceleration.y");
  const auto lin_accel_z_ind = indexVector(
    pal_statistics_names_, "state_interface.torso_imu_sensor/linear_acceleration.z");

  imu_orientation_ = Eigen::Quaterniond(
    msg->values[quat_w_index],
    msg->values[quat_x_index],
    msg->values[quat_y_index],
    msg->values[quat_z_index]);
  imu_received_ = true;

  Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
  Eigen::Vector3d gravity_body = imu_orientation_.normalized().conjugate() * gravity_world;

  const rclcpp::Time stamp(msg->header.stamp);
  for (auto & p : policies_) {
    if (p.obs_terms.find("base_ang_vel") != p.obs_terms.end()) {
      auto & term = p.obs_terms["base_ang_vel"];
      term->last_msg_time = stamp;
      term->current[0] = msg->values[ang_vel_x_ind];
      term->current[1] = msg->values[ang_vel_y_ind];
      term->current[2] = msg->values[ang_vel_z_ind];
    }

    if (p.obs_terms.find("base_lin_acc") != p.obs_terms.end()) {
      auto & term = p.obs_terms["base_lin_acc"];
      term->last_msg_time = stamp;
      term->current[0] = msg->values[lin_accel_x_ind];
      term->current[1] = msg->values[lin_accel_y_ind];
      term->current[2] = msg->values[lin_accel_z_ind];
    }

    if (p.obs_terms.find("imu_projected_gravity") != p.obs_terms.end()) {
      auto & term = p.obs_terms["imu_projected_gravity"];
      term->last_msg_time = stamp;
      term->current[0] = gravity_body.x();
      term->current[1] = gravity_body.y();
      term->current[2] = gravity_body.z();
    }

    if (p.obs_terms.find("base_lin_vel") != p.obs_terms.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "base_lin_vel term present but not populated from PAL statistics - need odometry source");
    }

    if (p.obs_terms.find("joint_pos") != p.obs_terms.end() &&
      p.obs_terms.find("joint_vel") != p.obs_terms.end())
    {
      auto & pos_term = p.obs_terms["joint_pos"];
      auto & vel_term = p.obs_terms["joint_vel"];
      pos_term->last_msg_time = stamp;
      vel_term->last_msg_time = stamp;
      for (size_t k = 0; k < p.config.joint_names.size(); ++k) {
        const std::string pos_key =
          "state_interface." + p.config.joint_names[k] + "/position";
        const std::string vel_key =
          "state_interface." + p.config.joint_names[k] + "/velocity";
        const auto jnt_pos_index = indexVector(pal_statistics_names_, pos_key);
        const auto jnt_vel_index = indexVector(pal_statistics_names_, vel_key);
        pos_term->current[k] = msg->values[jnt_pos_index] - p.config.default_joint_pos[k];
        vel_term->current[k] = msg->values[jnt_vel_index];
      }
    }
  }
  runInference();
}

// ---- LOAD A SINGLE POLICY INSTANCE ----

bool KangMjlabNode::loadPolicy(
  PolicyInstance & p, const std::string & task, const std::string & model_name)
{
  p.config.task = task;

#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 2)
  const std::string pkg_root =
    ament_index_cpp::get_package_share_path("pal_policy_deployer").string();
#else
  const std::string pkg_root =
    ament_index_cpp::get_package_share_directory("pal_policy_deployer");
#endif
  const std::string config_dir = pkg_root + "/models/" + task + "/" + task + "_" + model_name +
    "_mjlab/";

  RCLCPP_INFO(
    get_logger(), "[Policy '%s'] Looking for ONNX model in: %s",
    p.name.c_str(), config_dir.c_str());

  std::string model_path;
  try {
    model_path = KangMjlabModel::findOnnxModel(config_dir);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(get_logger(), "[Policy '%s'] %s", p.name.c_str(), e.what());
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "[Policy '%s'] Found ONNX model: %s",
    p.name.c_str(), model_path.c_str());

  if (!loadPolicyConfig(model_path, p)) {
    RCLCPP_FATAL(
      get_logger(),
      "[Policy '%s'] Failed to load policy configuration from ONNX metadata", p.name.c_str());
    return false;
  }

  if (task == "imitation") {
    p.motion.num_joints = static_cast<int>(p.config.joint_names.size());
    if (!computeHistoryLength(p.config, p.motion)) {
      RCLCPP_FATAL(
        get_logger(), "[Policy '%s'] Obs dim %d not divisible by single obs dim %d",
        p.name.c_str(), p.config.obs_dim, p.config.single_obs_dim);
      return false;
    }
  }

  if (!p.config.joint_stiffness.empty() && !p.config.joint_damping.empty()) {
    RCLCPP_INFO(get_logger(), "[Policy '%s'] Stiffness and damping gains are:", p.name.c_str());
    for (size_t i = 0; i < p.config.action_joint_names.size(); ++i) {
      const double stiffness = i < p.config.joint_stiffness.size() ? p.config.joint_stiffness[i] : 0.0;
      const double damping = i < p.config.joint_damping.size() ? p.config.joint_damping[i] : 0.0;
      RCLCPP_INFO(
        get_logger(), "  %s: {p: %f, i: 0.0, d: %f}",
        p.config.action_joint_names[i].c_str(), stiffness, damping);
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "[Policy '%s'] task=%s obs_dim=%d (single=%d x history=%d) action_dim=%d joints=%zu/%zu",
    p.name.c_str(), p.config.task.c_str(),
    p.config.obs_dim, p.config.single_obs_dim, p.config.history_length,
    p.config.action_dim, p.config.joint_names.size(), p.config.action_joint_names.size());
  if (task == "imitation") {
    RCLCPP_INFO(
      get_logger(),
      "[Policy '%s'] Motion: autoregressive from robot state, command_dim=%d",
      p.name.c_str(), p.motion.commandDim());
  }

  RCLCPP_INFO(get_logger(), "[Policy '%s'] Initializing observation terms:", p.name.c_str());
  if (!initializeObservationTerms(p)) {
    return false;
  }

  p.action.resize(p.config.action_dim, 0.0);
  p.zero_action.resize(p.config.action_dim, 0.0);

  return true;
}

}  // namespace pal_policy_inferer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pal_policy_inferer::KangMjlabNode>();
  if (!node->init()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::Rate rate(50.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
