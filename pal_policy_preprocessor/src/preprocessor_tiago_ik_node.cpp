// pal_policy_preprocessor_tiago_ik_node_main.cpp
//
// Non-component, ONNX-metadata-driven preprocessor that matches the *unautomated*
// node’s behavior as closely as possible, while staying automated.
//
// obs = [pos(N), vel(N), last_action(M), command(14)]
//
// Matching points vs unautomated:
//  - Same joint order (from ONNX metadata) and same default subtraction
//  - Grippers forced to default (0 after subtraction) every update
//  - last_action copied verbatim
//  - command layout (left pose then right pose, qw first)
//  - Seeds initial command pose to the same values as the unautomated constructor
//  - Velocity write condition matches unautomated: only write if velocity.size()==name.size()

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace fs = std::filesystem;
using std::placeholders::_1;

// -------------------- EXCLUDED (GRIPPER) JOINTS --------------------
static const std::vector<std::string> excluded_action_joints = {
  "gripper_right_outer_finger_right_joint", "gripper_right_outer_finger_left_joint",
  "gripper_left_outer_finger_right_joint", "gripper_left_outer_finger_left_joint",
};

static bool isExcludedJoint(const std::string & jn)
{
  return std::find(excluded_action_joints.begin(), excluded_action_joints.end(), jn) !=
         excluded_action_joints.end();
}

static bool isGripperDefaultJoint(const std::string & jn)
{
  return isExcludedJoint(jn);
}

// -------------------- CMD DIM --------------------
static constexpr int DIM_CMD = 14;

// -------------------- UTILS --------------------
static std::vector<std::string> splitString(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

static std::vector<double> parseDoubles(const std::string & str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      continue;
    }
    values.push_back(std::stod(token));
  }
  return values;
}

// -------------------- POLICY CONFIG --------------------
struct PolicyConfig
{
  // raw metadata
  std::string run_path;
  std::vector<std::string> joint_names;        // ALL joints in obs (incl grippers)
  std::vector<double> default_joint_pos;       // same length as joint_names

  std::vector<double> action_scale;            // optional; usually action_dim
  std::vector<double> joint_stiffness;         // optional
  std::vector<double> joint_damping;           // optional

  // derived
  std::vector<std::string> action_joint_names; // joint_names excluding grippers
  int obs_dim = 0;
  int action_dim = 0;
};

// -------------------- PARSE METADATA FROM ONNX MODEL --------------------
static bool loadPolicyConfigFromONNX(
  Ort::Env & env, const std::string & model_path, PolicyConfig & config,
  const rclcpp::Logger & logger)
{
  if (model_path.empty()) {
    RCLCPP_ERROR(logger, "onnx_model_path is empty");
    return false;
  }

  Ort::SessionOptions session_options;

  Ort::Session temp_session(nullptr);
  try {
    temp_session = Ort::Session(env, model_path.c_str(), session_options);
  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(logger, "Failed to open ONNX model '%s': %s", model_path.c_str(), e.what());
    return false;
  }

  Ort::ModelMetadata metadata = temp_session.GetModelMetadata();
  Ort::AllocatorWithDefaultOptions allocator;

  // Get all custom metadata keys
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

  // ---- Parse metadata ----
  if (metadata_map.count("run_path")) {
    config.run_path = metadata_map["run_path"];
  }

  if (metadata_map.count("joint_names")) {
    config.joint_names = splitString(metadata_map["joint_names"], ',');
  } else {
    RCLCPP_ERROR(logger, "Missing 'joint_names' in ONNX metadata");
    return false;
  }

  if (metadata_map.count("default_joint_pos")) {
    config.default_joint_pos = parseDoubles(metadata_map["default_joint_pos"]);
  } else {
    RCLCPP_ERROR(logger, "Missing 'default_joint_pos' in ONNX metadata");
    return false;
  }

  if (config.default_joint_pos.size() != config.joint_names.size()) {
    RCLCPP_ERROR(
      logger,
      "default_joint_pos length (%zu) != joint_names length (%zu)",
      config.default_joint_pos.size(), config.joint_names.size());
    return false;
  }

  if (metadata_map.count("action_scale")) {
    config.action_scale = parseDoubles(metadata_map["action_scale"]);
  }
  if (metadata_map.count("joint_stiffness")) {
    config.joint_stiffness = parseDoubles(metadata_map["joint_stiffness"]);
  }
  if (metadata_map.count("joint_damping")) {
    config.joint_damping = parseDoubles(metadata_map["joint_damping"]);
  }

  config.action_joint_names.clear();
  config.action_joint_names.reserve(config.joint_names.size());
  for (const auto & jn : config.joint_names) {
    if (!isExcludedJoint(jn)) {
      config.action_joint_names.push_back(jn);
    }
  }

  // Derive dims from METADATA + FIXED OBS STRUCTURE
  const int N = static_cast<int>(config.joint_names.size());
  const int M = static_cast<int>(config.action_joint_names.size());

  if (N <= 0 || M <= 0) {
    RCLCPP_ERROR(logger, "Invalid metadata sizes: N=%d joints, M=%d actions", N, M);
    return false;
  }

  config.action_dim = M;
  config.obs_dim = (2 * N) + M + DIM_CMD;

  if (!config.action_scale.empty() &&
    config.action_scale.size() != static_cast<size_t>(config.action_dim))
  {
    RCLCPP_WARN(
      logger,
      "action_scale size (%zu) != action_dim (%d). (Not fatal for preprocessor)",
      config.action_scale.size(), config.action_dim);
  }

  RCLCPP_INFO(
    logger,
    "Derived dims: N=%d joints, M=%d actions => obs_dim=%d, action_dim=%d",
    N, M, config.obs_dim, config.action_dim);

  // Optional debug prints (keep if you want)
  RCLCPP_INFO(logger, "=== action_joint_names (grippers excluded) ===");
  for (size_t i = 0; i < config.action_joint_names.size(); ++i) {
    RCLCPP_INFO(logger, "[%02zu] %s", i, config.action_joint_names[i].c_str());
  }

  RCLCPP_INFO(logger, "=== default_joint_pos (metadata order) ===");
  for (size_t i = 0; i < config.joint_names.size(); ++i) {
    RCLCPP_INFO(
      logger,
      "[%02zu] %-40s : %.6f",
      i,
      config.joint_names[i].c_str(),
      config.default_joint_pos[i]
    );
  }

  return true;
}

// -------------------- NODE --------------------
class IKObsPublisher : public rclcpp::Node
{
public:
  IKObsPublisher()
  : rclcpp::Node("pal_policy_preprocessor_tiago_ik_node"),
    env_(ORT_LOGGING_LEVEL_WARNING, "rl")
  {
    model_path_ = this->declare_parameter<std::string>("onnx_model_path", "");

    // Fallback discovery: find first .onnx under pal_policy_deployer share/models/tiagopro/exported
    if (model_path_.empty()) {
      std::string pkg_share =
        ament_index_cpp::get_package_share_directory("pal_policy_deployer");
      std::string config_dir =
        (fs::path(pkg_share) / "models" / "tiagopro" / "exported").string();

      bool found = false;
      if (fs::exists(config_dir) && fs::is_directory(config_dir)) {
        for (const auto & entry : fs::directory_iterator(config_dir)) {
          const auto & p = entry.path();
          if (entry.is_regular_file() && p.extension() == ".onnx") {
            model_path_ = p.string();
            found = true;
            RCLCPP_INFO(this->get_logger(), "Found ONNX model: %s", model_path_.c_str());
            break;
          }
        }
      }

      if (!found) {
        RCLCPP_FATAL(
          this->get_logger(),
          "onnx_model_path not set and no .onnx found in %s",
          config_dir.c_str());
        throw std::runtime_error("no onnx model found");
      }
    }

    // Load metadata + derive dims
    if (!loadPolicyConfigFromONNX(env_, model_path_, config_, this->get_logger())) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load ONNX metadata/config");
      throw std::runtime_error("metadata load failed");
    }

    // Allocate buffers based on config
    initBuffersFromConfig();
    buildJointIndexMap();

    // Subscriptions
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1, std::bind(&IKObsPublisher::jsCB, this, _1));

    sub_left_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/left_target_position", 1, std::bind(&IKObsPublisher::leftPosCB, this, _1));
    sub_left_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/left_target_orientation", 1, std::bind(&IKObsPublisher::leftQuatCB, this, _1));

    sub_right_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/right_target_position", 1, std::bind(&IKObsPublisher::rightPosCB, this, _1));
    sub_right_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/right_target_orientation", 1, std::bind(&IKObsPublisher::rightQuatCB, this, _1));

    sub_last_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ik_policy/last_action", 1, std::bind(&IKObsPublisher::lastCB, this, _1));

    pub_obs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ik_policy/obs", 1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&IKObsPublisher::onTimer, this));
  }

private:
  // -------------------- BUFFERS --------------------
  void initBuffersFromConfig()
  {
    if (config_.obs_dim <= 0 || config_.obs_dim > 200000) {
      throw std::runtime_error("obs_dim invalid/unreasonable: " + std::to_string(config_.obs_dim));
    }
    if (config_.action_dim <= 0 || config_.action_dim > 20000) {
      throw std::runtime_error(
              "action_dim invalid/unreasonable: " + std::to_string(config_.action_dim));
    }

    const size_t N = config_.joint_names.size();

    buff_joint_pos_.assign(N, 0.0);
    buff_joint_vel_.assign(N, 0.0);
    buff_last_action_.assign(static_cast<size_t>(config_.action_dim), 0.0);
    buff_command_.assign(DIM_CMD, 0.0);

    // ---- Match the unautomated node's initial command pose ----
    // Left pose (xyz, qw, qx, qy, qz)
    buff_command_[0] = 0.337962;
    buff_command_[1] = 0.203562;
    buff_command_[2] = 0.677366;
    buff_command_[3] = 0.078844;
    buff_command_[4] = 0.21089;
    buff_command_[5] = 0.00096864;
    buff_command_[6] = 0.97432;

    // Right pose (xyz, qw, qx, qy, qz)
    buff_command_[7] = 0.337751;
    buff_command_[8] = -0.202208;
    buff_command_[9] = 0.677405;
    buff_command_[10] = -0.078344;
    buff_command_[11] = 0.21066;
    buff_command_[12] = 0.0041078;
    buff_command_[13] = 0.97441;
  }

  void buildJointIndexMap()
  {
    joint_name_to_index_.clear();
    joint_name_to_index_.reserve(config_.joint_names.size());
    for (size_t i = 0; i < config_.joint_names.size(); ++i) {
      joint_name_to_index_[config_.joint_names[i]] = i;
    }
  }

  // -------------------- OBSERVATION VECTOR BUILDING --------------------
  // obs = [pos(N), vel(N), last_action(M), command(14)]
  std::vector<double> buildObservationVector() const
  {
    std::vector<double> obs;
    obs.reserve(static_cast<size_t>(config_.obs_dim));

    obs.insert(obs.end(), buff_joint_pos_.begin(), buff_joint_pos_.end());
    obs.insert(obs.end(), buff_joint_vel_.begin(), buff_joint_vel_.end());
    obs.insert(obs.end(), buff_last_action_.begin(), buff_last_action_.end());
    obs.insert(obs.end(), buff_command_.begin(), buff_command_.end());

    return obs;
  }

  // --------------------- CALLBACKS --------------------
  void jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.size() != msg->name.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "JointState invalid");
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);

    const bool has_full_velocity =
      (!msg->velocity.empty() && msg->velocity.size() == msg->name.size());

    // Fill joint position/velocity buffers in metadata order
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = joint_name_to_index_.find(msg->name[i]);
      if (it == joint_name_to_index_.end()) {
        continue;
      }

      const size_t k = it->second;

      // subtract default pose (from metadata)
      if (k < config_.default_joint_pos.size()) {
        buff_joint_pos_[k] = msg->position[i] - config_.default_joint_pos[k];
      }

      // Match unautomated behavior: only write velocities if velocity.size()==name.size()
      if (has_full_velocity) {
        buff_joint_vel_[k] = msg->velocity[i];
      }
    }

    // ---- Force grippers to DEFAULT every update ----
    for (size_t k = 0; k < config_.joint_names.size(); ++k) {
      const auto & jn = config_.joint_names[k];
      if (isGripperDefaultJoint(jn)) {
        // After subtracting default_joint_pos, "default" is 0.0 in obs
        buff_joint_pos_[k] = 0.0;
        buff_joint_vel_[k] = 0.0;
      }
    }
  }

  // ---- Target pose callbacks ----
  void leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    buff_command_[0] = msg->x;
    buff_command_[1] = msg->y;
    buff_command_[2] = msg->z;
  }

  void leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // policy expects (w, x, y, z)
    buff_command_[3] = msg->w;
    buff_command_[4] = msg->x;
    buff_command_[5] = msg->y;
    buff_command_[6] = msg->z;
  }

  void rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    buff_command_[7] = msg->x;
    buff_command_[8] = msg->y;
    buff_command_[9] = msg->z;
  }

  void rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // policy expects (w, x, y, z)
    buff_command_[10] = msg->w;
    buff_command_[11] = msg->x;
    buff_command_[12] = msg->y;
    buff_command_[13] = msg->z;
  }

  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    const size_t n = std::min(buff_last_action_.size(), a->data.size());
    for (size_t i = 0; i < n; ++i) {
      buff_last_action_[i] = a->data[i];
    }
  }

  // -------------------- TIMER CALLBACK --------------------
  void onTimer()
  {
    std_msgs::msg::Float64MultiArray m;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      m.data = buildObservationVector();

      if (static_cast<int>(m.data.size()) != config_.obs_dim) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Built obs size=%zu but expected obs_dim=%d",
          m.data.size(), config_.obs_dim);
        return;
      }
    }

    pub_obs_->publish(m);
  }

private:
  // ORT + config
  Ort::Env env_;
  std::string model_path_;
  PolicyConfig config_;

  // subscriptions/publisher
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  mutable std::mutex mtx_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  std::vector<double> buff_joint_pos_;    // size N
  std::vector<double> buff_joint_vel_;    // size N
  std::vector<double> buff_last_action_;  // size M
  std::vector<double> buff_command_;      // size 14
};

// -------------------- main() --------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKObsPublisher>());
  rclcpp::shutdown();
  return 0;
}
