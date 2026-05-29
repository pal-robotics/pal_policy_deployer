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
// Inference is triggered by /joint_states. All policies receive every sensor update
// so that switching is seamless (no stale observations).
//
// Subscribed topics
//   /joint_states                      sensor_msgs/JointState  (triggers inference)
//   /imu_sensor_broadcaster/imu        sensor_msgs/Imu
//   /cmd_vel                           geometry_msgs/Twist  (loco policies only)
//   /odom                              nav_msgs/Odometry    (only if base_lin_vel term is used)
//
// Published topics
//   /subscriber_controller/desired_state  sensor_msgs/JointState
//   pal_policy_deployer_statistics        PAL statistics (elapsed time per observation term)
//
// Action server
//   ~/switch_policy  pal_policy_inferer/action/SwitchPolicy
//
// Parameters
//   policies         string[]  list of policy names; each name namespaces its own params
//   <name>.task      string    "loco" or "imitation"
//   <name>.model     string    model identifier; selects models/<task>/<task>_<model>_mjlab/
//   <name>.frames    int       (imitation only) auto-return frame budget; 0 = use ONNX default
//   <name>.gains     string    impedance gains folder name (currently unused; reserved)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pal_statistics/pal_statistics_macros.hpp>

#include <pal_policy_inferer/action/switch_policy.hpp>

#include <onnxruntime_cxx_api.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

static const std::string POLICY_DEPLOYER_TOPIC = "pal_policy_deployer_statistics";

// Joints to exclude from action outputs (mechanically constrained / mimic).
static const std::vector<std::string> excluded_action_joints = {
    "leg_left_femur_joint", "leg_left_knee_joint",
    "leg_right_femur_joint", "leg_right_knee_joint"
};

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
  double elapsed_time;  // Since sensor update

  // stats_prefix disambiguates multiple policies registering the same term name
  ObservationTerm(
    rclcpp::Node * node,
    const std::string & n, int d, const std::string & stats_prefix = "")
  : name(n), dim(d), current(d, 0.0), last_msg_time(0, 0, RCL_ROS_TIME), elapsed_time(0.0)
  {
    const std::string full_name = "elapsed_time." + stats_prefix + name;
    REGISTER_VARIABLE(node, POLICY_DEPLOYER_TOPIC, full_name, &elapsed_time);
  }

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

// ---- AUTOREGRESSIVE MOTION STATE ----

struct MotionState
{
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  int num_joints = 0;
  int timestep = 0;
  int frames = 0;
  bool loaded = false;

  std::vector<double> getCommand() const
  {
    std::vector<double> cmd;
    cmd.reserve(num_joints * 2);
    cmd.insert(cmd.end(), joint_pos.begin(), joint_pos.end());
    cmd.insert(cmd.end(), joint_vel.begin(), joint_vel.end());
    return cmd;
  }

  int commandDim() const {return num_joints * 2;}

  void updateFromOutputs(
    const float * out_joint_pos, int nj,
    const float * out_joint_vel, int /*nj2*/)
  {
    for (int i = 0; i < nj && i < num_joints; ++i) {
      joint_pos[i] = out_joint_pos[i];
      joint_vel[i] = out_joint_vel[i];
    }
  }
};

// ---- POLICY CONFIGURATION ----

struct PolicyConfig
{
  std::string run_path;
  std::string task;  // "loco" or "imitation"
  std::vector<std::string> joint_names;
  std::vector<std::string> action_joint_names;
  std::vector<double> default_joint_pos;
  std::vector<double> action_offset;
  std::vector<double> action_scale;
  std::vector<double> joint_stiffness;
  std::vector<double> joint_damping;
  std::vector<std::string> observation_names;
  int obs_dim = 0;
  int action_dim = 0;
  int history_length = 0;
  int single_obs_dim = 0;
  int num_inputs = 0;
  int num_outputs = 0;
  int out_idx_actions = -1;
  int out_idx_joint_pos = -1;
  int out_idx_joint_vel = -1;
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

  std::unique_ptr<Ort::Session> sess;
  std::vector<const char *> in_names;
  std::vector<const char *> out_names;

  std::vector<int64_t> input_shape;
  std::vector<int64_t> time_step_shape;
  std::vector<double> action;
  std::vector<double> zero_action;
};

// ---- UTILITY FUNCTIONS ----

static std::vector<std::string> splitString(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

static std::vector<double> parseDoubles(const std::string & str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    values.push_back(std::stod(token));
  }
  return values;
}

static bool isExcludedJoint(const std::string & joint_name)
{
  return std::find(excluded_action_joints.begin(), excluded_action_joints.end(), joint_name)
         != excluded_action_joints.end();
}


// ---- MAIN NODE ----

class KangMjlabNode : public rclcpp::Node
{
public:
  KangMjlabNode()
  : rclcpp::Node("pal_policy_deployer"),
    ort_env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
    ort_mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
  }

  // Two-phase init so we can use shared_from_this() for the action server.
  bool init()
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
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&KangMjlabNode::jsCB, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_sensor_broadcaster/imu", 1,
      std::bind(&KangMjlabNode::imuCB, this, std::placeholders::_1));
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      std::bind(&KangMjlabNode::velCmdCB, this, std::placeholders::_1));

    pub_sub_cmd_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>(
      "/subscriber_controller/desired_state", 1);

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
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&KangMjlabNode::odomCB, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Subscribed to /odom for base_lin_vel");
    }

    RCLCPP_INFO(
      get_logger(), "Policy deployer started. Active policy: [0] '%s'",
      policies_[0].name.c_str());
    if (policies_[0].config.task == "imitation") {
      RCLCPP_INFO(get_logger(), "Waiting for joint state to initialize motion...");
    }

    return true;
  }

  ~KangMjlabNode() override
  {
    // The ONNX allocator strdups the input/output names; release them.
    Ort::AllocatorWithDefaultOptions alloc;
    for (auto & p : policies_) {
      for (auto * n : p.in_names) {
        if (n) {alloc.Free(const_cast<char *>(n));}
      }
      for (auto * n : p.out_names) {
        if (n) {alloc.Free(const_cast<char *>(n));}
      }
      p.in_names.clear();
      p.out_names.clear();
    }
  }

private:
  // ---- INITIALIZE MOTION FROM ROBOT STATE ----

  bool initializeMotionFromRobot(PolicyInstance & p)
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

  // ---- OBSERVATION TERM DIMENSION CALCULATOR ----

  static int getObservationTermDim(
    const PolicyConfig & cfg, const MotionState & mot, const std::string & term_name)
  {
    if (term_name == "base_ang_vel") {return 3;}
    if (term_name == "base_lin_vel") {return 3;}
    if (term_name == "base_lin_acc") {return 3;}
    if (term_name == "imu_projected_gravity") {return 3;}
    if (term_name == "joint_pos") {return static_cast<int>(cfg.joint_names.size());}
    if (term_name == "joint_vel") {return static_cast<int>(cfg.joint_names.size());}
    if (term_name == "actions") {return static_cast<int>(cfg.action_joint_names.size());}
    if (term_name == "command") {
      return (cfg.task == "imitation") ? mot.commandDim() : 3;
    }
    return -1;
  }

  // ---- OBSERVATION TERM INITIALIZATION ----

  bool initializeObservationTerms(PolicyInstance & p)
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

  // ---- ONNX METADATA PARSING ----

  bool loadPolicyConfig(
    const std::string & model_path, PolicyConfig & config, MotionState & motion)
  {
    Ort::SessionOptions session_options;
    Ort::Session temp_session(ort_env_, model_path.c_str(), session_options);

    Ort::ModelMetadata metadata = temp_session.GetModelMetadata();
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<Ort::AllocatedStringPtr> keys =
      metadata.GetCustomMetadataMapKeysAllocated(allocator);
    std::map<std::string, std::string> metadata_map;
    for (auto & key_ptr : keys) {
      std::string key = key_ptr.get();
      Ort::AllocatedStringPtr value_ptr =
        metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
      if (value_ptr) {
        metadata_map[key] = value_ptr.get();
      }
    }

    if (metadata_map.find("run_path") != metadata_map.end()) {
      config.run_path = metadata_map["run_path"];
    }

    auto require = [&](const char * key) {
        if (metadata_map.find(key) == metadata_map.end()) {
          RCLCPP_FATAL(get_logger(), "Missing '%s'", key);
          return false;
        }
        return true;
      };
    if (!require("joint_names")) {return false;}
    config.joint_names = splitString(metadata_map["joint_names"], ',');
    if (!require("default_joint_pos")) {return false;}
    config.default_joint_pos = parseDoubles(metadata_map["default_joint_pos"]);
    if (!require("action_scale")) {return false;}
    config.action_scale = parseDoubles(metadata_map["action_scale"]);
    if (!require("observation_names")) {return false;}
    config.observation_names = splitString(metadata_map["observation_names"], ',');

    if (metadata_map.find("joint_stiffness") != metadata_map.end()) {
      config.joint_stiffness = parseDoubles(metadata_map["joint_stiffness"]);
    }
    if (metadata_map.find("joint_damping") != metadata_map.end()) {
      config.joint_damping = parseDoubles(metadata_map["joint_damping"]);
    }

    config.action_joint_names.clear();
    config.action_offset.clear();
    for (size_t i = 0; i < config.joint_names.size(); ++i) {
      if (!isExcludedJoint(config.joint_names[i])) {
        config.action_joint_names.push_back(config.joint_names[i]);
        config.action_offset.push_back(config.default_joint_pos[i]);
      }
    }

    config.num_inputs = static_cast<int>(temp_session.GetInputCount());
    config.num_outputs = static_cast<int>(temp_session.GetOutputCount());

    {
      Ort::TypeInfo info = temp_session.GetInputTypeInfo(0);
      auto tensor_info = info.GetTensorTypeAndShapeInfo();
      config.obs_dim = static_cast<int>(tensor_info.GetShape()[1]);
    }

    for (int i = 0; i < config.num_outputs; ++i) {
      Ort::AllocatedStringPtr name = temp_session.GetOutputNameAllocated(i, allocator);
      std::string n = name.get();

      Ort::TypeInfo info = temp_session.GetOutputTypeInfo(i);
      auto tensor_info = info.GetTensorTypeAndShapeInfo();
      auto dims = tensor_info.GetShape();

      std::stringstream shape_ss;
      for (size_t j = 0; j < dims.size(); j++) {
        if (j > 0) {shape_ss << ",";}
        shape_ss << dims[j];
      }
      RCLCPP_INFO(
        get_logger(), "  Output %d: '%s' [%s]",
        i, n.c_str(), shape_ss.str().c_str());

      if (n == "actions") {
        config.out_idx_actions = i;
        config.action_dim = static_cast<int>(dims[1]);
      } else if (n == "joint_pos") {
        config.out_idx_joint_pos = i;
      } else if (n == "joint_vel") {
        config.out_idx_joint_vel = i;
      }
    }

    for (int i = 0; i < config.num_inputs; ++i) {
      Ort::AllocatedStringPtr name = temp_session.GetInputNameAllocated(i, allocator);
      Ort::TypeInfo info = temp_session.GetInputTypeInfo(i);
      auto tensor_info = info.GetTensorTypeAndShapeInfo();
      auto dims = tensor_info.GetShape();
      std::stringstream shape_ss;
      for (size_t j = 0; j < dims.size(); j++) {
        if (j > 0) {shape_ss << ",";}
        shape_ss << dims[j];
      }
      RCLCPP_INFO(
        get_logger(), "  Input %d: '%s' [%s]",
        i, name.get(), shape_ss.str().c_str());
    }

    // For imitation, single_obs_dim is deferred until motion.num_joints is known.
    if (config.task != "imitation") {
      config.single_obs_dim = 0;
      for (const auto & term_name : config.observation_names) {
        config.single_obs_dim += getObservationTermDim(config, motion, term_name);
      }

      if (config.single_obs_dim > 0 && config.obs_dim % config.single_obs_dim == 0) {
        config.history_length = config.obs_dim / config.single_obs_dim;
      } else {
        RCLCPP_FATAL(
          get_logger(), "Observation dimension %d not divisible by single obs dim %d",
          config.obs_dim, config.single_obs_dim);
        return false;
      }
    }

    if (config.action_dim != static_cast<int>(config.action_joint_names.size())) {
      RCLCPP_WARN(
        get_logger(), "Action dim mismatch: ONNX=%d, expected=%zu",
        config.action_dim, config.action_joint_names.size());
    }

    return true;
  }

  // ---- BUILD OBSERVATION VECTOR ----

  std::vector<float> buildObservationVector(PolicyInstance & p)
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

  void updateAllHistory(PolicyInstance & p)
  {
    for (auto & pair : p.obs_terms) {
      pair.second->updateHistory();
    }
  }

  // ---- INFERENCE ----

  void runInference()
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
      }

      updateAllHistory(p);
      obs_input = buildObservationVector(p);
    }
    PUBLISH_STATISTICS(this, POLICY_DEPLOYER_TOPIC);

    std::vector<Ort::Value> input_tensors;

    Ort::Value obs_tensor = Ort::Value::CreateTensor<float>(
      ort_mem_, obs_input.data(), obs_input.size(),
      p.input_shape.data(), p.input_shape.size());
    input_tensors.push_back(std::move(obs_tensor));

    if (p.config.num_inputs > 1 && p.config.task == "imitation") {
      Ort::Value ts_tensor = Ort::Value::CreateTensor<float>(
        ort_mem_, &time_step_val, 1,
        p.time_step_shape.data(), p.time_step_shape.size());
      input_tensors.push_back(std::move(ts_tensor));
    }

    auto outputs = p.sess->Run(
      Ort::RunOptions{nullptr},
      p.in_names.data(), input_tensors.data(),
      input_tensors.size(),
      p.out_names.data(), p.out_names.size());

    const float * actions_data = outputs[p.config.out_idx_actions].GetTensorData<float>();
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
      if (p.config.out_idx_joint_pos >= 0 && p.config.out_idx_joint_vel >= 0) {
        const float * out_jp = outputs[p.config.out_idx_joint_pos].GetTensorData<float>();
        const float * out_jv = outputs[p.config.out_idx_joint_vel].GetTensorData<float>();
        p.motion.updateFromOutputs(out_jp, p.motion.num_joints, out_jv, p.motion.num_joints);
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
        // TODO impedance: reapply gains on auto-switch-back to policy [0].
      }
    }

    control_msgs::msg::MultiDOFCommand cmd_msg;
    cmd_msg.dof_names = p.config.action_joint_names;
    cmd_msg.values = p.action;
    cmd_msg.values_dot = p.zero_action;
    pub_sub_cmd_->publish(cmd_msg);
  }

  // ---- POLICY SWITCH ACTION ----

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const SwitchPolicy::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received switch_policy request: index=%d", goal->policy_index);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<SwitchPolicyGoalHandle> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handleAccepted(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle)
  {
    std::thread{std::bind(&KangMjlabNode::executeSwitch, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void executeSwitch(const std::shared_ptr<SwitchPolicyGoalHandle> goal_handle)
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

    // TODO impedance: reapply gains for the newly active policy.

    result->success = true;
    result->message = "Switched to policy [" + std::to_string(new_idx) + "] '" +
      policies_[new_idx].name + "'";
    goal_handle->succeed(result);
  }

  // ---- SENSOR CALLBACKS ----
  // Each callback updates ALL policies so that switching is seamless (no stale obs).

  void jsCB(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    latest_joint_state_ = *msg;
    if (!joint_state_received_) {
      joint_state_received_ = true;
    }

    for (auto & p : policies_) {
      if (p.obs_terms.find("joint_pos") != p.obs_terms.end()) {
        auto & term = p.obs_terms["joint_pos"];
        term->last_msg_time = msg->header.stamp;
        for (size_t k = 0; k < p.config.joint_names.size(); ++k) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == p.config.joint_names[k]) {
              term->current[k] = msg->position[i] - p.config.default_joint_pos[k];
              break;
            }
          }
        }
      }

      if (p.obs_terms.find("joint_vel") != p.obs_terms.end()) {
        auto & term = p.obs_terms["joint_vel"];
        term->last_msg_time = msg->header.stamp;
        for (size_t k = 0; k < p.config.joint_names.size(); ++k) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == p.config.joint_names[k]) {
              term->current[k] = msg->velocity[i];
              break;
            }
          }
        }
      }
    }

    if (!imu_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 5000,
        "Waiting for IMU data before running inference...");
      return;
    }
    runInference();
  }

  void velCmdCB(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
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

  void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
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

  void imuCB(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_orientation_ = Eigen::Quaterniond(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);
    imu_received_ = true;

    Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
    Eigen::Vector3d gravity_body = imu_orientation_.conjugate() * gravity_world;

    const rclcpp::Time stamp(msg->header.stamp);
    for (auto & p : policies_) {
      if (p.obs_terms.find("base_ang_vel") != p.obs_terms.end()) {
        auto & term = p.obs_terms["base_ang_vel"];
        term->last_msg_time = stamp;
        term->current[0] = msg->angular_velocity.x;
        term->current[1] = msg->angular_velocity.y;
        term->current[2] = msg->angular_velocity.z;
      }

      if (p.obs_terms.find("base_lin_acc") != p.obs_terms.end()) {
        auto & term = p.obs_terms["base_lin_acc"];
        term->last_msg_time = stamp;
        term->current[0] = msg->linear_acceleration.x;
        term->current[1] = msg->linear_acceleration.y;
        term->current[2] = msg->linear_acceleration.z;
      }

      if (p.obs_terms.find("imu_projected_gravity") != p.obs_terms.end()) {
        auto & term = p.obs_terms["imu_projected_gravity"];
        term->last_msg_time = stamp;
        term->current[0] = gravity_body.x();
        term->current[1] = gravity_body.y();
        term->current[2] = gravity_body.z();
      }
    }
  }

  // ---- LOAD A SINGLE POLICY INSTANCE ----

  bool loadPolicy(PolicyInstance & p, const std::string & task, const std::string & model_name)
  {
    p.config.task = task;

    const std::string pkg_root =
      ament_index_cpp::get_package_share_directory("pal_policy_deployer");
    const std::string config_dir = pkg_root + "/models/" + task + "/" + task + "_" + model_name +
      "_mjlab/";

    RCLCPP_INFO(
      get_logger(), "[Policy '%s'] Looking for ONNX model in: %s",
      p.name.c_str(), config_dir.c_str());

    std::string model_path;
    bool found = false;
    if (!fs::is_directory(config_dir)) {
      RCLCPP_FATAL(
        get_logger(), "[Policy '%s'] Model directory does not exist: %s",
        p.name.c_str(), config_dir.c_str());
      return false;
    }
    for (auto & entry : fs::directory_iterator(config_dir)) {
      fs::path fp = entry.path();
      if (fs::is_regular_file(fp) && fp.extension() == ".onnx") {
        model_path = fp.string();
        found = true;
        RCLCPP_INFO(
          get_logger(), "[Policy '%s'] Found ONNX model: %s",
          p.name.c_str(), model_path.c_str());
        break;
      }
    }
    if (!found) {
      RCLCPP_FATAL(
        get_logger(), "[Policy '%s'] No ONNX model found in %s",
        p.name.c_str(), config_dir.c_str());
      return false;
    }

    if (!loadPolicyConfig(model_path, p.config, p.motion)) {
      RCLCPP_FATAL(
        get_logger(),
        "[Policy '%s'] Failed to load policy configuration from ONNX metadata", p.name.c_str());
      return false;
    }

    if (task == "imitation") {
      p.motion.num_joints = static_cast<int>(p.config.joint_names.size());
      p.config.single_obs_dim = 0;
      for (const auto & term_name : p.config.observation_names) {
        p.config.single_obs_dim += getObservationTermDim(p.config, p.motion, term_name);
      }
      if (p.config.single_obs_dim > 0 && p.config.obs_dim % p.config.single_obs_dim == 0) {
        p.config.history_length = p.config.obs_dim / p.config.single_obs_dim;
      } else {
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
    p.input_shape = {1, static_cast<int64_t>(p.config.obs_dim)};
    p.time_step_shape = {1, 1};

    p.sess.reset(new Ort::Session(ort_env_, model_path.c_str(), Ort::SessionOptions{}));

    Ort::AllocatorWithDefaultOptions alloc;
    for (size_t i = 0; i < p.sess->GetInputCount(); ++i) {
      p.in_names.push_back(p.sess->GetInputNameAllocated(i, alloc).release());
    }
    for (size_t i = 0; i < p.sess->GetOutputCount(); ++i) {
      p.out_names.push_back(p.sess->GetOutputNameAllocated(i, alloc).release());
    }

    return true;
  }

  // ---- MEMBERS ----

  Ort::Env ort_env_;
  Ort::MemoryInfo ort_mem_;

  std::vector<PolicyInstance> policies_;
  std::atomic<int> active_policy_idx_{0};

  // Shared robot state (physical robot, independent of policy)
  Eigen::Quaterniond imu_orientation_{1.0, 0.0, 0.0, 0.0};
  bool imu_received_{false};
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_received_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr pub_sub_cmd_;

  rclcpp_action::Server<SwitchPolicy>::SharedPtr switch_action_server_;
};

}  // namespace pal_policy_inferer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pal_policy_inferer::KangMjlabNode>();
  if (!node->init()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
