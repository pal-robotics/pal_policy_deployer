#include "pal_policy_preprocessor/preprocessor_tiago_ik_node.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

using std::placeholders::_1;

// obs layout (67):
// positions [0..18]  : torso(1), left arm(7), left grip(2 default), right arm(7), right grip(2 default)
// velocities [19..37]: same order
// actions   [38..52] : 15 (left7, right7, torso1) raw policy outputs (as before)
// commands  [53..66] : left pose 7 + right pose 7  (xyz + qwxyz for each)

static constexpr int POS0 = 0;
static constexpr int VEL0 = 19;
static constexpr int ACT0 = 38;
static constexpr int CMDL0 = 53;   // left command starts here
static constexpr int CMDR0 = 60;   // right command starts here

// indices within pos/vel blocks:
static constexpr int POS_TORSO = 0;          // obs[0]
static constexpr int POS_LA0 = 1;            // obs[1..7]
static constexpr int POS_LG0 = 8;            // obs[8..9]  (default)
static constexpr int POS_RA0 = 10;           // obs[10..16]
static constexpr int POS_RG0 = 17;           // obs[17..18] (default)

static constexpr int VEL_TORSO = 0;          // obs[19]
static constexpr int VEL_LA0 = 1;            // obs[20..26]
static constexpr int VEL_LG0 = 8;            // obs[27..28] default
static constexpr int VEL_RA0 = 10;           // obs[29..35]
static constexpr int VEL_RG0 = 17;           // obs[36..37] default

// ---- DEFAULT JOINT POSITIONS ----
static const double torso_default = 0.1;

static const std::vector<double> gripper_default = {0.0, 0.0};

static const std::vector<double> left_default = {
  0.358, -1.827, 0.470, -2.341, 0.000, -1.201, 0.000
};

static const std::vector<double> right_default = {
  -0.358, -1.827, -0.470, -2.341, 0.000, -1.201, 0.000
};

// Names to read from joint states
static const std::vector<std::string> torso_and_arms_order = {
  "torso_lift_joint",
  "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
  "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
  "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint",
  "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint",
};

// Map k (0..14) into obs indices for position/velocity blocks
static inline int pos_index_for_k(int k)
{
  if (k == 0) {
    return POS0 + POS_TORSO;             // torso -> obs[0]

  }
  // left arm k=1..7 -> obs[1..7]
  if (k >= 1 && k <= 7) {return POS0 + POS_LA0 + (k - 1);}

  // right arm k=8..14 -> obs[10..16]
  return POS0 + POS_RA0 + (k - 8);
}

static inline int vel_index_for_k(int k)
{
  if (k == 0) {
    return VEL0 + VEL_TORSO;             // torso -> obs[19]

  }
  // left arm k=1..7 -> obs[20..26]
  if (k >= 1 && k <= 7) {return VEL0 + VEL_LA0 + (k - 1);}

  // right arm k=8..14 -> obs[29..35]
  return VEL0 + VEL_RA0 + (k - 8);
}

IKObsPublisher::IKObsPublisher()
: rclcpp::Node("onnx_input_publisher"), obs_(67, 0.0)   // <-- 67 now
{
  // ---- Default gripper states in obs (positions + velocities) ----
  // positions
  obs_[POS0 + POS_LG0 + 0] = gripper_default[0];
  obs_[POS0 + POS_LG0 + 1] = gripper_default[1];
  obs_[POS0 + POS_RG0 + 0] = gripper_default[0];
  obs_[POS0 + POS_RG0 + 1] = gripper_default[1];
  // velocities
  obs_[VEL0 + VEL_LG0 + 0] = 0.0;
  obs_[VEL0 + VEL_LG0 + 1] = 0.0;
  obs_[VEL0 + VEL_RG0 + 0] = 0.0;
  obs_[VEL0 + VEL_RG0 + 1] = 0.0;

  // === Initial left command pose (xyz, qw, qx, qy, qz) ===
  obs_[CMDL0 + 0] = 0.337962;
  obs_[CMDL0 + 1] = 0.203562;
  obs_[CMDL0 + 2] = 0.677366;
  obs_[CMDL0 + 3] = 0.078844;
  obs_[CMDL0 + 4] = 0.21089;
  obs_[CMDL0 + 5] = 0.00096864;
  obs_[CMDL0 + 6] = 0.97432;

  // === Initial right command pose (xyz, qw, qx, qy, qz) ===
  obs_[CMDR0 + 0] = 0.337751;
  obs_[CMDR0 + 1] = -0.202208;
  obs_[CMDR0 + 2] = 0.677405;
  obs_[CMDR0 + 3] = -0.078344;
  obs_[CMDR0 + 4] = 0.21066;
  obs_[CMDR0 + 5] = 0.0041078;
  obs_[CMDR0 + 6] = 0.97441;

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

// JointState callback
void IKObsPublisher::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.empty() || msg->position.size() != msg->name.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "JointState invalid");
    return;
  }

  // For torso+arms only (15 joints)
  for (size_t k = 0; k < torso_and_arms_order.size(); ++k) {
    const auto & wanted = torso_and_arms_order[k];

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == wanted) {
        double q = msg->position[i];

        // ---- subtract default pose ----
        if (k == 0) {
          q -= torso_default;
        } else if (k >= 1 && k <= 7) {
          q -= left_default[k - 1];
        } else if (k >= 8 && k <= 14) {
          q -= right_default[k - 8];
        }

        // write position
        obs_[pos_index_for_k(static_cast<int>(k))] = q;

        // write velocity if present
        if (!msg->velocity.empty() && msg->velocity.size() == msg->name.size()) {
          obs_[vel_index_for_k(static_cast<int>(k))] = msg->velocity[i];
        }

        break;
      }
    }
  }

  // Ensure grippers remain default every update
  obs_[POS0 + POS_LG0 + 0] = gripper_default[0];
  obs_[POS0 + POS_LG0 + 1] = gripper_default[1];
  obs_[POS0 + POS_RG0 + 0] = gripper_default[0];
  obs_[POS0 + POS_RG0 + 1] = gripper_default[1];

  obs_[VEL0 + VEL_LG0 + 0] = 0.0;
  obs_[VEL0 + VEL_LG0 + 1] = 0.0;
  obs_[VEL0 + VEL_RG0 + 0] = 0.0;
  obs_[VEL0 + VEL_RG0 + 1] = 0.0;
}

// ---- Target pose callbacks ----
void IKObsPublisher::leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  obs_[CMDL0 + 0] = msg->x;
  obs_[CMDL0 + 1] = msg->y;
  obs_[CMDL0 + 2] = msg->z;
}

void IKObsPublisher::leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  obs_[CMDL0 + 3] = msg->w;  // policy expects (w,x,y,z)
  obs_[CMDL0 + 4] = msg->x;
  obs_[CMDL0 + 5] = msg->y;
  obs_[CMDL0 + 6] = msg->z;
}

void IKObsPublisher::rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  obs_[CMDR0 + 0] = msg->x;
  obs_[CMDR0 + 1] = msg->y;
  obs_[CMDR0 + 2] = msg->z;
}

void IKObsPublisher::rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  obs_[CMDR0 + 3] = msg->w;  // policy expects (w,x,y,z)
  obs_[CMDR0 + 4] = msg->x;
  obs_[CMDR0 + 5] = msg->y;
  obs_[CMDR0 + 6] = msg->z;
}

void IKObsPublisher::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
{
  // actions (15) at obs_[38..52]
  const size_t N = std::min<std::size_t>(15, a->data.size());
  for (size_t i = 0; i < N; ++i) {
    obs_[ACT0 + i] = a->data[i];
  }
}

void IKObsPublisher::onTimer()
{
  std_msgs::msg::Float64MultiArray m;
  m.data = obs_;
  pub_obs_->publish(m);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKObsPublisher>());
  rclcpp::shutdown();
  return 0;
}
