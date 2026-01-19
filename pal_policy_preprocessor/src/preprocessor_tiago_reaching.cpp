#include "pal_policy_preprocessor/preprocessor_tiago_reaching.hpp"

#include <algorithm>

using std::placeholders::_1;

static constexpr int DIM_CMD = 14;

static const std::vector<std::string> excluded_action_joints = {
  "gripper_right_outer_finger_right_joint",
  "gripper_right_outer_finger_left_joint",
  "gripper_left_outer_finger_right_joint",
  "gripper_left_outer_finger_left_joint",
};

TiagoReaching::TiagoReaching(const rclcpp::NodeOptions & options)
: Preprocessor("tiago_reaching_preprocessor", options)
{
  const std::string model_path = resolveModelPath();
  buildMetadataMap(model_path);

  parseMetadataIntoConfig(metadataMap());
  initBuffersFromConfig();
  buildJointIndexMap();

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&TiagoReaching::jsCB, this, _1));

  sub_left_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/left_target_position", 1, std::bind(&TiagoReaching::leftPosCB, this, _1));
  sub_left_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "/left_target_orientation", 1, std::bind(&TiagoReaching::leftQuatCB, this, _1));

  sub_right_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/right_target_position", 1, std::bind(&TiagoReaching::rightPosCB, this, _1));
  sub_right_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "/right_target_orientation", 1, std::bind(&TiagoReaching::rightQuatCB, this, _1));

  sub_last_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/reaching_tiago/last_action", 1, std::bind(&TiagoReaching::lastCB, this, _1));

  run(std::chrono::milliseconds(20));
}

std::vector<double> TiagoReaching::buildObservationVector() const
{
  std::vector<double> obs;
  obs.reserve(static_cast<size_t>(config_.obs_dim));

  obs.insert(obs.end(), buff_joint_pos_.begin(), buff_joint_pos_.end());
  obs.insert(obs.end(), buff_joint_vel_.begin(), buff_joint_vel_.end());
  obs.insert(obs.end(), buff_last_action_.begin(), buff_last_action_.end());
  obs.insert(obs.end(), buff_command_.begin(), buff_command_.end());

  return obs;
}

void TiagoReaching::parseMetadataIntoConfig(
  const std::map<std::string, std::string> & mm)
{
  config_.joint_names = splitString(mm.at("joint_names"), ',');
  config_.default_joint_pos = parseDoubles(mm.at("default_joint_pos"));
  config_.action_scale = parseDoubles(mm.at("action_scale"));


  for (const auto & jn : config_.joint_names) {
    if (!isExcludedJoint(jn, excluded_action_joints)) {
      config_.action_joint_names.push_back(jn);
    }
  }

  const int N = static_cast<int>(config_.joint_names.size());
  const int M = static_cast<int>(config_.action_joint_names.size());

  config_.action_dim = M;
  config_.obs_dim = (2 * N) + M + DIM_CMD; // TO DO >> can we infer this from ONNX model?
}

void TiagoReaching::initBuffersFromConfig()
{
  const size_t N = config_.joint_names.size();

  buff_joint_pos_ = config_.default_joint_pos;
  buff_joint_vel_.assign(N, 0.0);
  buff_last_action_.assign(static_cast<size_t>(config_.action_dim), 0.0);
  buff_command_.assign(DIM_CMD, 0.0);
}

void TiagoReaching::buildJointIndexMap()
{
  for (size_t i = 0; i < config_.joint_names.size(); ++i) {
    joint_name_to_index_[config_.joint_names[i]] = i;
  }
}

void TiagoReaching::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = joint_name_to_index_.find(msg->name[i]);
    if (it == joint_name_to_index_.end()) {
      continue;
    }

    const size_t k = it->second;
    buff_joint_pos_[k] = msg->position[i] - config_.default_joint_pos[k];

    if (msg->velocity.size() == msg->name.size()) {
      buff_joint_vel_[k] = msg->velocity[i];
    }
  }
}

void TiagoReaching::leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  buff_command_[0] = msg->x;
  buff_command_[1] = msg->y;
  buff_command_[2] = msg->z;
}

void TiagoReaching::leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  buff_command_[3] = msg->w;
  buff_command_[4] = msg->x;
  buff_command_[5] = msg->y;
  buff_command_[6] = msg->z;
}

void TiagoReaching::rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  buff_command_[7] = msg->x;
  buff_command_[8] = msg->y;
  buff_command_[9] = msg->z;
}

void TiagoReaching::rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  buff_command_[10] = msg->w;
  buff_command_[11] = msg->x;
  buff_command_[12] = msg->y;
  buff_command_[13] = msg->z;
}

void TiagoReaching::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  const size_t n = std::min(buff_last_action_.size(), msg->data.size());
  for (size_t i = 0; i < n; ++i) {
    buff_last_action_[i] = msg->data[i];
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TiagoReaching>());
  rclcpp::shutdown();
  return 0;
}
