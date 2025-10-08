#include <ros/ros.h>
#include <ros/package.h>
// ros msgs
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
// is it all needed ??
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <onnxruntime_cxx_api.h>
#include <boost/filesystem.hpp>
// core libraries
#include <vector>
#include <algorithm> 
#include <array>
#include <mutex>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fs = boost::filesystem;

// ---- OBSERVATION INFORMATIONS ----

static const std::vector<std::string> obs_order {
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
static const std::vector<double> obs_offset {
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

// ---- ACTION INFORMATIONS ----

static const std::vector<std::string> action_order {
  "pelvis_1_joint", 
  "leg_left_1_joint", "leg_right_1_joint", 
  "pelvis_2_joint", 
  "leg_left_2_joint", "leg_right_2_joint", 
  "arm_left_1_joint", "arm_right_1_joint", 
  "leg_left_3_joint", "leg_right_3_joint", 
  "arm_left_2_joint", "arm_right_2_joint", 
  "leg_left_length_joint", /*"leg_left_femur_joint",*/
  "leg_right_length_joint", /*"leg_right_femur_joint",*/ 
  "arm_left_3_joint", "arm_right_3_joint", 
  "leg_left_4_joint", /*"leg_left_knee_joint",*/  
  "leg_right_4_joint", /*"leg_right_knee_joint",*/  
  "arm_left_4_joint", "arm_right_4_joint", 
  "leg_left_5_joint", "leg_right_5_joint"
};
static const std::vector<double> action_offset {
  0.0, 
  0.0, 0.0, 
  0.0, 
  0.05, 0.05, 
  0.24, -0.24, 
  0.0, 0.0, 
  1.32, 1.32, 
  0.6832, // 1.1172, 
  0.6832, // 1.1172, 
  1.57, -1.57, 
  -0.05, // 2.2345, 
  -0.05, // 2.2345, 
  0.8, 0.8, 
  0.0, 0.0
};
static const std::vector<double> action_scale {
  0.3125, 
  0.5, 0.5, 
  0.625, 
  0.575, 0.575, 
  0.1075, 0.1075, 
  0.3475, 0.3475, 
  0.1075, 0.1075, 
  0.25,
  0.25, 
  0.1075, 0.1075, 
  0.35, 
  0.35, 
  0.1075, 0.1075, 
  0.5125, 0.5125
};

// Policy specific vars

Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"rl");
std::unique_ptr<Ort::Session> sess;
Ort::MemoryInfo mem(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault));
std::vector<const char*> in_names, out_names;

Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
Eigen::Vector3d projected_gravity_b = Eigen::Vector3d::Zero();

std::vector<double> observation(83, 0.0);
std::mutex observation_mutex;

static constexpr int64_t input_shape[2] = {1, 83};
std_msgs::Float64MultiArray cmd, last;

std::vector<double> action(22);
std::vector<double> zero_action(22, 0.0);

ros::Publisher pub_sub_cmd;

void timerCallback(const ros::TimerEvent&)
{
  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex);
    obs_input.assign(observation.begin(), observation.end());   // 86 values
  }

  Ort::Value input_tensor =
      Ort::Value::CreateTensor<float>(mem, obs_input.data(), obs_input.size(), input_shape, 2);

  auto outputs = sess->Run(Ort::RunOptions{nullptr},
                           in_names.data(),  &input_tensor, 1,
                           out_names.data(), 1);

  const float* y_f = outputs.front().GetTensorData<float>();

  for (size_t i = 0; i < 22; ++i) {
    action[i] = action_scale[i] * y_f[i] + action_offset[i];
    observation[52+i] = y_f[i];
  }

  sensor_msgs::JointState cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();

  cmd_msg.name = action_order;
  cmd_msg.position = action;
  cmd_msg.velocity = zero_action;
  cmd_msg.effort = zero_action;

  pub_sub_cmd.publish(cmd_msg);
}

// OBSERVATION CONSTRUCTION 

// joint pos and joint vel (26 + 26 = 52)
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (size_t k = 0; k < obs_order.size(); ++k) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == obs_order[k]) {
        observation[k] = msg->position[i] - obs_offset[k];
        observation[k+26] = msg->velocity[i] * 0.05;
        // ROS_INFO("Isaac Lab name -> %s | joint_state name -> %s", obs_order[k].c_str(), msg->name[i].c_str());
      }
    }
  }
}

// imu data (6)
void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    // base_ang_vel
    observation[74] = msg->angular_velocity.x * 0.1;
    observation[75] = msg->angular_velocity.y * 0.1;
    observation[76] = msg->angular_velocity.z * 0.1;
    // ROS_INFO("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // base_lin_acc
    observation[77] = msg->linear_acceleration.x * 0.1;
    observation[78] = msg->linear_acceleration.y * 0.1;
    observation[79] = msg->linear_acceleration.z * 0.1;
    // ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

// base state data (9)
// void baseStateCB(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     // base linear velocity
//     observation[74] = msg->twist.twist.linear.x;
//     observation[75] = msg->twist.twist.linear.y;
//     observation[76] = msg->twist.twist.linear.z;
//     ROS_INFO("Base angular velocity: x: %f, y: %f, z: %f\n", observation[74], observation[75], observation[76]);

//     // base angular velocity
//     observation[77] = msg->twist.twist.angular.x;
//     observation[78] = msg->twist.twist.angular.y;
//     observation[79] = msg->twist.twist.angular.z;
//     ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", observation[77], observation[78], observation[79]);

//     // projected gravity
//     base_quat.w() = msg->pose.pose.orientation.w;
//     base_quat.x() = msg->pose.pose.orientation.x;
//     base_quat.y() = msg->pose.pose.orientation.y;
//     base_quat.z() = msg->pose.pose.orientation.z;

//     // Rotate world gravity into body frame
//     projected_gravity_b = base_quat.conjugate() * gravity_world;
//     // might need to normalize but im really unsure as it does not seem to be done in isaac lab
//     // projected_gravity_b.normalize();  // unit vector

//     observation[81] = projected_gravity_b.x();
//     observation[82] = projected_gravity_b.y();
//     observation[83] = projected_gravity_b.z();
//     ROS_INFO("Base linear acceleration: x: %f, y: %f, z: %f\n", observation[81], observation[82], observation[83]);
// }

// rosrun teleop_twist_keyboard teleop_twist_keyboard.py (3)
void velCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    observation[80] = msg->linear.x;
    observation[81] = msg->linear.y;
    observation[82] = msg->angular.z;

    ROS_INFO("Velocity command: Vx: %f, Vy: %f, wz: %f\n", msg->linear.x, msg->linear.y, msg->angular.z);
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"pal_policy_deployer"); ros::NodeHandle nh("~");
  
  std::string pkg_root = ros::package::getPath("pal_policy_deployer");
  std::string config_dir = pkg_root + "/models/loco/loco_v36/exported/";
  std::string model_path;
  bool found = false;
  for (auto& entry : fs::directory_iterator(config_dir)) {
    fs::path p = entry.path();
    if (fs::is_regular_file(p) && p.extension() == ".onnx") {
      model_path = entry.path().string();
      found = true;
      ROS_INFO("Found ONNX model: %s", model_path.c_str());
      break;
    }
  }
  if (!found) {
    ROS_FATAL("No ONNX model found in %s, and fallback '%s' doesn't exist.",
              config_dir.c_str(), model_path.c_str());
    return 1;
  }

  sess.reset(new Ort::Session(env,model_path.c_str(), Ort::SessionOptions{}));

  in_names .push_back(sess->GetInputNameAllocated (0, Ort::AllocatorWithDefaultOptions()).release());
  out_names.push_back(sess->GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions()).release());

  ros::Subscriber s1 = nh.subscribe("/joint_states", 1, jsCB);
  ros::Subscriber s2 = nh.subscribe("/base_imu", 1, imuCB);
//   ros::Subscriber s2 = nh.subscribe("/floating_base_pose_simulated", 1, baseStateCB);
  ros::Subscriber s3 = nh.subscribe("/cmd_vel", 1, velCmdCB);

  pub_sub_cmd = nh.advertise<sensor_msgs::JointState>("/subscriber_controller/desired_state", 1);
  
  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();
}
