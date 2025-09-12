#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <ros/package.h>            // for ros::package::getPath()
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// #include <onnxruntime_cxx_api.h>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <vector>

ros::Publisher pub_group_cmd;
ros::Publisher pub_ruckig_goal;
ros::Publisher pub_traj_cmd;
ros::Publisher pub_sub_cmd;
ros::Publisher pub_last;

std::vector<double> last_action(22, 0.0);  // 72 obs, 22 actions
std::mutex last_action_mutex;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return v < lo ? lo : (hi < v ? hi : v);
}


static const std::vector<std::string> full_body_order {
    "pelvis_1_joint", 
    "leg_left_1_joint", "leg_right_1_joint", 
    "pelvis_2_joint", 
    "leg_left_2_joint", "leg_right_2_joint", 
    "arm_left_1_joint", "arm_right_1_joint", 
    "leg_left_3_joint", "leg_right_3_joint", 
    "arm_left_2_joint", "arm_right_2_joint", 
    "leg_left_length_joint", "leg_right_length_joint",
    "arm_left_3_joint", "arm_right_3_joint", 
    "leg_left_4_joint", "leg_right_4_joint", 
    "arm_left_4_joint", "arm_right_4_joint", 
    "leg_left_5_joint", "leg_right_5_joint"
};


// offset in the order of Isaac Lab
static const std::vector<double> joint_pos_offset {
    0.0, 
    0.0, 0.0, 
    0.0, 
    0.05, 0.05, 
    0.0, 0.0, 
    0.0, 0.0, 
    1.24, 1.24, 
    0.6, 0.6,
    1.57, -1.57, 
    -0.07, -0.07, 
    1.25, 1.25, 
    0.0, 0.0
};

// add this in a timer callback running at 50Hz
void timerCallback(const ros::TimerEvent&)
{
  /* ---- copy the latest last_action under lock ------------------------- */
  std::vector<float> y_f;
  {
    std::lock_guard<std::mutex> lk(last_action_mutex);
    y_f.assign(last_action.begin(), last_action.end());   // 22 values
  }

  /* ---- publish results ------------------------------------------------ */
  std_msgs::Float64MultiArray cmd, last;
  const float action_scale = 0.5;

  std::vector<double> action(22);
  // for (size_t i = 0; i < 22; ++i)
  //     action[i] = action_scale * static_cast<double>(y_f[i]) + joint_pos_offset_legs[i];

  action = {
    // left leg joints pos target
    action_scale*y_f[1],
    action_scale*y_f[4] + 0.05,
    action_scale*y_f[6],
    action_scale*y_f[8] + 0.6832,
    action_scale*y_f[10] - 0.05,
    action_scale*y_f[12],

    // right leg joints pos target
    action_scale*y_f[2],
    action_scale*y_f[5] + 0.05,
    action_scale*y_f[7],
    action_scale*y_f[9] + 0.6832,
    action_scale*y_f[11] - 0.05,
    action_scale*y_f[13],

    action_scale*y_f[0], // pelvis 1 joint
    action_scale*y_f[3], // pelvis 2 joint

    0.24,
    1.32,
    1.57,
    0.8,

    -0.24,
    1.32,
    -1.57,
    0.8,
  };


  //////////////////////////////////////////////////////////////////////////////
  ///////////////////  Publish SubscriberController command  ///////////////////
  //////////////////////////////////////////////////////////////////////////////

  sensor_msgs::JointState cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.name = {
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_length_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",

    "leg_right_1_joint",
    "leg_right_2_joint",
    "leg_right_3_joint",
    "leg_right_length_joint",
    "leg_right_4_joint",
    "leg_right_5_joint",

    "pelvis_1_joint",
    "pelvis_2_joint",

    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",

    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
  };

  cmd_msg.position = action;

  std::vector<double> res(22, 0.0);

  cmd_msg.velocity = res;
  cmd_msg.effort = res;

  pub_sub_cmd.publish(cmd_msg);

}


void lastActionCB(const std_msgs::Float64MultiArray::ConstPtr& m)
{
  if (m->data.size() != 22) return;

  std::lock_guard<std::mutex> lock(last_action_mutex);
  last_action.assign(m->data.begin(), m->data.end());
  printf("Received last action\n");

}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"pal_policy_postprocessor"); ros::NodeHandle nh("~");
  
  pub_sub_cmd = nh.advertise<sensor_msgs::JointState>("/subscriber_controller/desired_state",1);
  
  ros::Subscriber sub = nh.subscribe("/rl_policy/last_action",1,lastActionCB);

  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();
}
