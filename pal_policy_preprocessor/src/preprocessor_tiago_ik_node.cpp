#include "pal_policy_preprocessor/preprocessor_tiago_ik_node.hpp"
using std::placeholders::_1;

IKObsPublisher::IKObsPublisher()
: rclcpp::Node("onnx_input_publisher"), obs_(59, 0.0)
{
  // initial target (x, y, z)
  // Initial left command pose (xyz, qw, qx, qy, qz)
  obs_[45] = 0.27634;   // left x
  obs_[46] = 0.20833;  // left y
  obs_[47] = 0.54071;   // left z

  obs_[48] = 0.078844f;  // left qw
  obs_[49] = 0.21089f;    // left qx
  obs_[50] = 0.00096864f;  // left qy
  obs_[51] = 0.97432f;    // left qz

  // Initial right command pose (xyz, qw, qx, qy, qz) – fill with something reasonable or zeros
  obs_[52] = 0.27627;  // right x
  obs_[53] = -0.20836;  // right y
  obs_[54] = 0.54072;  // right z

  obs_[55] = -0.078344;  // right qw
  obs_[56] = 0.21066;  // right qx
  obs_[57] = 0.0041078;  // right qy
  obs_[58] = 0.97441;  // right qz


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

  pub_obs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_policy/obs", 1);

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&IKObsPublisher::onTimer, this));
}


void IKObsPublisher::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // TO DO: add correct ones
  obs_[0] = msg->position[18];   // torso

  obs_[1] = msg->position[0];
  obs_[2] = msg->position[1];
  obs_[3] = msg->position[2];
  obs_[4] = msg->position[3];
  obs_[5] = msg->position[4];
  obs_[6] = msg->position[5];
  obs_[7] = msg->position[6];   // left arm (7 joints)

  obs_[8] = msg->position[7];
  obs_[9] = msg->position[8];
  obs_[10] = msg->position[9];
  obs_[11] = msg->position[10];
  obs_[12] = msg->position[11];
  obs_[13] = msg->position[12];
  obs_[14] = msg->position[13]; // right arm (7 joints)

  // --- Joint velocities: obs_[15..29] ---
  obs_[15] = msg->velocity[18];  // torso

  obs_[16] = msg->velocity[0];
  obs_[17] = msg->velocity[1];
  obs_[18] = msg->velocity[2];
  obs_[19] = msg->velocity[3];
  obs_[20] = msg->velocity[4];
  obs_[21] = msg->velocity[5];
  obs_[22] = msg->velocity[6];  // left arm

  obs_[23] = msg->velocity[7];
  obs_[24] = msg->velocity[8];
  obs_[25] = msg->velocity[9];
  obs_[26] = msg->velocity[10];
  obs_[27] = msg->velocity[11];
  obs_[28] = msg->velocity[12];
  obs_[29] = msg->velocity[13]; // right arm
}

void IKObsPublisher::leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Left command position -> obs_[45..47]
  obs_[45] = msg->x;
  obs_[46] = msg->y;
  obs_[47] = msg->z;
}

void IKObsPublisher::leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  // ROS gives (x,y,z,w), policy expects (w,x,y,z)
  // Left command orientation -> obs_[48..51]
  obs_[48] = msg->w;
  obs_[49] = msg->x;
  obs_[50] = msg->y;
  obs_[51] = msg->z;
}

void IKObsPublisher::rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Right command position -> obs_[52..54]
  obs_[52] = msg->x;
  obs_[53] = msg->y;
  obs_[54] = msg->z;
}

void IKObsPublisher::rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  // ROS gives (x,y,z,w), policy expects (w,x,y,z)
  // Right command orientation -> obs_[55..58]
  obs_[55] = msg->w;
  obs_[56] = msg->x;
  obs_[57] = msg->y;
  obs_[58] = msg->z;
}


void IKObsPublisher::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
{
  // Actions for 15 joints -> obs_[30..44]
  // (first torso, then left arm, then right arm)
  for (size_t i = 0; i < 15 && i < a->data.size(); ++i) {
    obs_[30 + i] = a->data[i];
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
