#include "pal_policy_preprocessor/preprocessor_tiago_reaching.hpp"

#include <algorithm>

using std::placeholders::_1;

static constexpr int DIM_CMD = 14;

TiagoReaching::TiagoReaching(const rclcpp::NodeOptions & options)
: Preprocessor("tiago_reaching_preprocessor", options)
{
  const std::string joint_names_str = this->declare_parameter<std::string>("joint_names", "");
  const std::string default_joint_pos_str = this->declare_parameter<std::string>(
    "default_joint_pos", "");

  const int64_t input_dim = this->declare_parameter<int64_t>("input_dimension", 0);
  const int64_t output_dim = this->declare_parameter<int64_t>("output_dimension", 0);

  config_.joint_names = splitString(joint_names_str, ',');
  config_.default_joint_pos = parseDoubles(default_joint_pos_str);

  config_.input_dim = static_cast<size_t>(input_dim);
  config_.output_dim = static_cast<size_t>(output_dim);

  left_pos_ =
    this->declare_parameter<std::vector<double>>(
    "initial_pose_left.position", std::vector<double>{});

  left_quat_ =
    this->declare_parameter<std::vector<double>>(
    "initial_pose_left.orientation", std::vector<double>{});

  right_pos_ =
    this->declare_parameter<std::vector<double>>(
    "initial_pose_right.position", std::vector<double>{});

  right_quat_ =
    this->declare_parameter<std::vector<double>>(
    "initial_pose_right.orientation", std::vector<double>{});

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


  obs_.assign(static_cast<size_t>(config_.input_dim), 0.0);
  run(std::chrono::milliseconds(20));
}

void TiagoReaching::buildInputVector(std::vector<double> * obs) const
{
  auto it = obs->begin();
  it = std::copy(buff_joint_pos_.begin(), buff_joint_pos_.end(), it);
  it = std::copy(buff_joint_vel_.begin(), buff_joint_vel_.end(), it);
  it = std::copy(buff_last_action_.begin(), buff_last_action_.end(), it);
  it = std::copy(buff_command_.begin(), buff_command_.end(), it);
}

void TiagoReaching::initBuffersFromConfig()
{
  const size_t N = config_.joint_names.size();
  buff_joint_pos_ = config_.default_joint_pos;
  buff_joint_vel_.assign(N, 0.0);
  buff_last_action_.assign(static_cast<size_t>(config_.output_dim), 0.0);
  // ---- Initial command pose ----
  buff_command_.resize(DIM_CMD);

  // Left pose (xyz, qw, qx, qy, qz)
  buff_command_[0] = left_pos_[0];
  buff_command_[1] = left_pos_[1];
  buff_command_[2] = left_pos_[2];
  buff_command_[3] = left_quat_[0];
  buff_command_[4] = left_quat_[1];
  buff_command_[5] = left_quat_[2];
  buff_command_[6] = left_quat_[3];

  // Right pose (xyz, qw, qx, qy, qz)
  buff_command_[7] = right_pos_[0];
  buff_command_[8] = right_pos_[1];
  buff_command_[9] = right_pos_[2];
  buff_command_[10] = right_quat_[0];
  buff_command_[11] = right_quat_[1];
  buff_command_[12] = right_quat_[2];
  buff_command_[13] = right_quat_[3];
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
