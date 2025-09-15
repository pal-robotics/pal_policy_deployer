#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
 #include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <ros/package.h>            // for ros::package::getPath()
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <onnxruntime_cxx_api.h>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <vector>

Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"rl");
std::unique_ptr<Ort::Session> sess;
Ort::MemoryInfo mem(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault));
std::vector<const char*> in_names, out_names;

ros::Publisher pub_group_cmd;
ros::Publisher pub_ruckig_goal;
ros::Publisher pub_traj_cmd;
ros::Publisher pub_sub_cmd;
ros::Publisher pub_last;

std::vector<double> observation(72, 0.0);  // 72 obs, 22 actions
std::mutex observation_mutex;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return v < lo ? lo : (hi < v ? hi : v);
}

static const std::vector<std::string> isaaclab_order {
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

// offset in the order of joint_states joint names
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


// 1 if this joint uses y_f, 0 if it’s an arm (fixed offset only)
static const std::vector<int> use_y = {
  1, 1, 1, 1, 1, 1,
  0, 0,
  1, 1,
  0, 0,
  1, 1,
  0, 0,
  1, 1,
  0, 0 
};

// Index in y_f if used, otherwise 0
static const std::vector<int> y_idx = {
   0, 1, 2, 3, 4, 5,
   0, 0,
   6, 7,
   0, 0,
   8, 9,
   0, 0,
   10, 11,
   0, 0 
};

static constexpr int64_t input_shape[2] = {1, 72};
std_msgs::Float64MultiArray cmd, last;
const float action_scale = 0.5;

std::vector<double> action(22);

std::vector<double> res(22, 0.0);

// add this in a timer callback running at 50Hz
void timerCallback(const ros::TimerEvent&)
{
  /* ---- copy the latest observation under lock ------------------------- */
  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex);
    obs_input.assign(observation.begin(), observation.end());   // 72 values
  }

  /* ---- wrap it in an Ort::Value tensor -------------------------------- */

  Ort::Value input_tensor =
      Ort::Value::CreateTensor<float>(mem, obs_input.data(), obs_input.size(), input_shape, 2);

  /* ---- run inference -------------------------------------------------- */
  auto outputs = sess->Run(Ort::RunOptions{nullptr},
                           in_names.data(),  &input_tensor, 1,
                           out_names.data(), 1);

  const float* y_f = outputs.front().GetTensorData<float>();

  for (size_t i = 0; i < 22; ++i) {
    action[i] = action_scale * y_f[y_idx[i]] * use_y[i] + joint_pos_offset[i];
  }

  sensor_msgs::JointState cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();

  cmd_msg.name = isaaclab_order;
  cmd_msg.position = action;
  cmd_msg.velocity = res;
  cmd_msg.effort = res;

  pub_sub_cmd.publish(cmd_msg);

  last.data.resize(14);
  for (size_t i = 0; i < 14; ++i)
      last.data[i] = y_f[i];
  pub_last.publish(last);
}


void obsCB(const std_msgs::Float64MultiArray::ConstPtr& m)
{
  if (m->data.size() != 72) return;

  std::lock_guard<std::mutex> lock(observation_mutex);
  observation.assign(m->data.begin(), m->data.end());

}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"pal_policy_deployer"); ros::NodeHandle nh("~");
  
  std::string pkg_root = ros::package::getPath("pal_policy_deployer");
  std::string config_dir = pkg_root + "/models/balance/exported/";
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

  // 2) If none found, fail
  if (!found) {
    ROS_FATAL("No ONNX model found in %s, and fallback '%s' doesn't exist.",
              config_dir.c_str(), model_path.c_str());
    return 1;
  }

  sess.reset(new Ort::Session(env,model_path.c_str(),Ort::SessionOptions{}));

  in_names .push_back(sess->GetInputNameAllocated (0,Ort::AllocatorWithDefaultOptions()).release());
  out_names.push_back(sess->GetOutputNameAllocated(0,Ort::AllocatorWithDefaultOptions()).release());

  pub_sub_cmd = nh.advertise<sensor_msgs::JointState>("/subscriber_controller/desired_state",1); 
  pub_last = nh.advertise<std_msgs::Float64MultiArray>("/rl_policy/last_action",1);
  ros::Subscriber sub = nh.subscribe("/rl_policy/obs",1,obsCB);

  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();
}
