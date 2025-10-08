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
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

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

std::vector<double> obs(86, 0.0);

Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
Eigen::Vector3d projected_gravity_b = Eigen::Vector3d::Zero();


// joint pos and joint vel (26 + 26 = 52)
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (size_t k = 0; k < isaaclab_order.size(); ++k) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == isaaclab_order[k]) {
        obs[k] = msg->position[i] - joint_pos_offset[k];
        obs[k+26] = msg->velocity[i]; // * 0.05;
        // ROS_INFO("Isaac Lab name -> %s | joint_state name -> %s", isaaclab_order[k].c_str(), msg->name[i].c_str());
      }
    }
  }
}

// last action (22)
void lastCB(const std_msgs::Float64MultiArray::ConstPtr& a)
{
  for (size_t i = 0; i < 22 && i < a->data.size(); ++i)
    obs[52 + i] = a->data[i];
}

// // imu data (6)
// void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     // base_ang_vel
//     obs[66] = msg->angular_velocity.x;
//     obs[67] = msg->angular_velocity.y;
//     obs[68] = msg->angular_velocity.z;
//     // ROS_INFO("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

//     // base_lin_acc
//     obs[69] = msg->linear_acceleration.x;
//     obs[70] = msg->linear_acceleration.y;
//     obs[71] = msg->linear_acceleration.z;
//     // ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
// }

// base state data (9)
void baseStateCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    // base linear velocity
    obs[74] = msg->twist.twist.linear.x;
    obs[75] = msg->twist.twist.linear.y;
    obs[76] = msg->twist.twist.linear.z;
    // ROS_INFO("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // base angular velocity
    obs[77] = msg->twist.twist.angular.x;
    obs[78] = msg->twist.twist.angular.y;
    obs[79] = msg->twist.twist.angular.z;
    // ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // projected gravity
    base_quat.w() = msg->pose.pose.orientation.w;
    base_quat.x() = msg->pose.pose.orientation.x;
    base_quat.y() = msg->pose.pose.orientation.y;
    base_quat.z() = msg->pose.pose.orientation.z;

    // Rotate world gravity into body frame
    projected_gravity_b = base_quat.conjugate() * gravity_world;
    // might need to normalize but im really unsure as it does not seem to be done in isaac lab
    // projected_gravity_b.normalize();  // unit vector

    obs[81] = projected_gravity_b.x();
    obs[82] = projected_gravity_b.y();
    obs[83] = projected_gravity_b.z();
    // ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

// rosrun teleop_twist_keyboard teleop_twist_keyboard.py (3)
void velCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    obs[84] = msg->linear.x;
    obs[85] = msg->linear.y;
    obs[86] = msg->angular.z;

    // ROS_INFO("Velocity command: Vx: %f, Vy: %f, wz: %f\n", msg->linear.x, msg->linear.y, msg->angular.z);
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"onnx_input_pub"); ros::NodeHandle nh;

  ros::Subscriber s1 = nh.subscribe("/joint_states", 1, jsCB);
  ros::Subscriber s2 = nh.subscribe("/rl_policy/last_action", 1, lastCB);
  ros::Subscriber s3 = nh.subscribe("/subscriber_controller/actual_base_state", 1, baseStateCB);
  ros::Subscriber s4 = nh.subscribe("/cmd_vel", 1, velCmdCB);

  ros::Publisher  pub= nh.advertise<std_msgs::Float64MultiArray>("/rl_policy/obs", 1);

  ros::Rate r(50);
  while(ros::ok()){
    std_msgs::Float64MultiArray m; m.data = obs; pub.publish(m);
    ros::spinOnce(); r.sleep();
  }
}
