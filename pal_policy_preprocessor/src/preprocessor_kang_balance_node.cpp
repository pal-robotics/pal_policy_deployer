#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <algorithm> 
#include <array>

// ----------- FULL BODY INFORMATIONS ----------------------------

// Isaac Lab joint order
static const std::vector<std::string> isaaclab_order {
  "pelvis_1_joint", 
  "leg_left_1_joint", "leg_right_1_joint", 
  "pelvis_2_joint", 
  "leg_left_2_joint", "leg_right_2_joint", 
  "arm_left_1_joint", "arm_right_1_joint", 
  "leg_left_3_joint", "leg_right_3_joint", 
  "arm_left_2_joint", "arm_right_2_joint", 
  "leg_left_length_joint", "leg_left_femur_joint", 
  "leg_right_length_joint", "leg_right_femur_joint", 
  "arm_left_3_joint", "arm_right_3_joint", 
  "leg_left_4_joint", "leg_left_knee_joint", 
  "leg_right_4_joint", "leg_right_knee_joint", 
  "arm_left_4_joint", "arm_right_4_joint", 
  "leg_left_5_joint", "leg_right_5_joint",
};

// Isaac Lab joint position offsets
static const std::vector<double> joint_pos_offset {
  0.0, 
  0.0, 0.0, 
  0.0, 
  0.05, 0.05, 
  0.24, -0.24, 
  0.0, 0.0, 
  1.32, 1.32, 
  0.6832, 1.1172, 
  0.6832, 1.1172, 
  1.57, -1.57, 
  -0.05, 2.2345, 
  -0.05, 2.2345, 
  0.8, 0.8, 
  0.0, 0.0
};

std::vector<double> obs(72, 0.0);

// joint pos and joint vel (26 + 26 = 52)
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (size_t k = 0; k < isaaclab_order.size(); ++k) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == isaaclab_order[k]) {
        obs[k] = msg->position[i] - joint_pos_offset[k];
        obs[k+26] = msg->velocity[i];
        // ROS_INFO("Isaac Lab name -> %s | joint_state name -> %s", isaaclab_order[k].c_str(), msg->name[i].c_str());
      }
    }
  }
}


// last action (14)
void lastCB(const std_msgs::Float64MultiArray::ConstPtr& a)
{
  for (size_t i = 0; i < 14 && i < a->data.size(); ++i)
    obs[52 + i] = a->data[i];
}

// compute imu data
// (6)
void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{

    // base_ang_vel
    obs[66] = msg->angular_velocity.x;
    obs[67] = msg->angular_velocity.y;
    obs[68] = msg->angular_velocity.z;
    // ROS_INFO("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // base_lin_acc
    obs[69] = msg->linear_acceleration.x;
    obs[70] = msg->linear_acceleration.y;
    obs[71] = msg->linear_acceleration.z;
    // ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"onnx_input_pub"); ros::NodeHandle nh;

  ros::Subscriber s1 = nh.subscribe("/joint_states", 1, jsCB);
  ros::Subscriber s2 = nh.subscribe("/rl_policy/last_action", 1, lastCB);
  ros::Subscriber s3 = nh.subscribe("/base_imu", 1, imuCB);

  ros::Publisher  pub= nh.advertise<std_msgs::Float64MultiArray>("/rl_policy/obs", 1);

  ros::Rate r(50);
  while(ros::ok()){
    std_msgs::Float64MultiArray m; m.data = obs; pub.publish(m);
    ros::spinOnce(); r.sleep();
  }
}