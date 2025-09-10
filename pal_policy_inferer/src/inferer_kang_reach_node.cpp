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

std::vector<double> observation(65, 0.0);  // 65 obs, 6 actions
std::mutex observation_mutex;

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return v < lo ? lo : (hi < v ? hi : v);
}

// add this in a timer callback running at 50Hz
void timerCallback(const ros::TimerEvent&)
{
  /* ---- copy the latest observation under lock ------------------------- */
  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex);
    obs_input.assign(observation.begin(), observation.end());   // 65 values
  }

  /* ---- wrap it in an Ort::Value tensor -------------------------------- */
  static constexpr int64_t input_shape[2] = {1, 65};
  Ort::Value input_tensor =
      Ort::Value::CreateTensor<float>(mem, obs_input.data(), obs_input.size(), input_shape, 2);

  /* ---- run inference -------------------------------------------------- */
  auto outputs = sess->Run(Ort::RunOptions{nullptr},
                           in_names.data(),  &input_tensor, 1,
                           out_names.data(), 1);

  const float* y_f = outputs.front().GetTensorData<float>();

  /* ---- publish results ------------------------------------------------ */
  std_msgs::Float64MultiArray cmd, last;
  const float action_scale = 0.5;

  std::vector<double> action = {
    action_scale*y_f[0],
    action_scale*y_f[1] + 0.05,
    action_scale*y_f[2],
    action_scale*y_f[4] - 0.05,
    action_scale*y_f[5],
    action_scale*y_f[3] + 0.6832,
  };
  // printf("Target joint position:\n joint 1: %f,\n joint 2: %f,\n joint 3: %f,\n joint 4: %f,\n joint 5: %f,\n joint length: %f,\n", action[0], action[1], action[2], action[3], action[4], action[5]);
  // printf("Non processed target joint position:\n joint 1: %f,\n joint 2: %f,\n joint 3: %f,\n joint 4: %f,\n joint 5: %f,\n joint length: %f,\n", y_f[0], y_f[1], y_f[2], y_f[5], y_f[3], y_f[4]);

  std::vector<double> min_val = {
    -0.2617993877991494,  
    -0.7417649320975901,  
    -0.47123889803846897,
    0.131973,
    -0.7417649,
    -0.4712389,
  };

  std::vector<double> max_val = {
    0.6981317007977318,  
    0.6632251157578453,
    0.47123889803846897,
    0.71416639,
    0.6632251,
    0.4712389
  };

  std::vector<double> clamped_action(6);

  for (size_t i = 0; i < 6; ++i) {
      clamped_action[i] = clamp(action[i], min_val[i], max_val[i]);
  }


  ////////////////////////////////////////////////////////////////////////////
  //////////////  Publish JointGorupPositionController command  //////////////
  ////////////////////////////////////////////////////////////////////////////
  // cmd.data = {
  //   action[0],
  //   action[1],
  //   action[2],
  //   action[3],
  // };
  // pub_group_cmd.publish(cmd);

  //////////////////////////////////////////////////////////////////////////
  //////////////  Publish JointTrajectoryController command  //////////////
  //////////////////////////////////////////////////////////////////////////

  trajectory_msgs::JointTrajectory traj;
  traj.header.stamp = ros::Time::now();
  traj.joint_names = {
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_length_joint",
  };

  // scale & clamp your actions exactly as before,
  // but now fill positions[6]
  std::vector<double> positions = {
    clamped_action[0],
    clamped_action[1],
    clamped_action[2],
    clamped_action[3],
    clamped_action[4],
    clamped_action[5],
  };


  // std::vector<double> positions = {
  //   action[0],
  //   action[1],
  //   action[2],
  //   action[3],
  //   action[4],
  //   action[5],
  // };

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = positions;
  pt.time_from_start = ros::Duration(1.0);  // 1s to reach; adjust as you like

  traj.points.push_back(pt);
  pub_traj_cmd.publish(traj);

  //////////////////////////////////////////////////////////////////////////////
  ///////////////////  Publish SubscriberController command  ///////////////////
  //////////////////////////////////////////////////////////////////////////////

  // ANKLE JOINTS NOT ACTIONNED

  // sensor_msgs::JointState cmd_msg;
  // cmd_msg.header.stamp = ros::Time::now();
  // cmd_msg.name = {
  //   "leg_left_1_joint",
  //   "leg_left_2_joint",
  //   "leg_left_3_joint",
  //   "leg_left_length_joint",
  // };
  // cmd_msg.position = {action[0], action[1], action[2], action[3]}; // {0.0, 0.0, 0.0, 0.0, 0.6}; //
  // cmd_msg.velocity = {0.0, 0.0, 0.0, 0.0};
  // cmd_msg.effort = {0.0, 0.0, 0.0, 0.0};
  // pub_sub_cmd.publish(cmd_msg);


  // // ALL JOINTS OF THE LEFT LEG ACTIONNED

  sensor_msgs::JointState cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.name = {
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_length_joint",
  };
  // cmd_msg.position = {action[0], action[1], action[2], action[3], 0.0, 0.0}; // {0.0, 0.0, 0.0, 0.0, 0.6}; //

  cmd_msg.position = {action[0], action[1], action[2], action[3], action[4], action[5]}; // {0.0, 0.0, 0.0, 0.0, 0.6};
  // cmd_msg.position = {0.0, 0.0, 0.0, 0.6, 0.0, 0.0};
  cmd_msg.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  cmd_msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  pub_sub_cmd.publish(cmd_msg);

  //////////////////////////////////////////////////////////////////////////////
  ///////////////////  Publish Ruckig command  /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // std::vector<double> positions_ruckig = {
  //   action[0],
  //   action[1],
  //   action[2],
  //   action[3],
  // };

  // // std::vector<double> positions_ruckig = {
  // //   0.0,
  // //   0.0,
  // //   0.0,
  // //   0.6
  // // };



  // std_msgs::Float64MultiArray ruckig_goal;
  // ruckig_goal.data = positions_ruckig;

  // pub_ruckig_goal.publish(ruckig_goal);

  //////////////////////////////////////////////////////////////////////////
  last.data.resize(6);
  for (size_t i = 0; i < 6; ++i)
      last.data[i] = y_f[i];
  pub_last.publish(last);
}


void obsCB(const std_msgs::Float64MultiArray::ConstPtr& m)
{
  if (m->data.size() != 65) return;

  std::lock_guard<std::mutex> lock(observation_mutex);
  observation.assign(m->data.begin(), m->data.end());

}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"onnx_inference"); ros::NodeHandle nh("~");
  

  double start_delay;
  nh.param("start_delay", start_delay, 0.5);
  ROS_INFO("Waiting %.1f seconds before starting...", start_delay);

  // either ROS Duration:
  ros::Duration(start_delay).sleep();

  std::string pkg_root = ros::package::getPath("onnx_inference_cpp");
  std::string config_dir = pkg_root + "/src/models/kang_reach/reach_v7/exported/";
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

  // 2) If none found, fall back to the param (or fail)
  if (!found) {
    nh.param("model_path", model_path,
      std::string("/home/louislelay/exchange/rl_ws/src/onnx_inference_cpp/src/models/kang_reach/reach_v4_muj_rand/exported/policy.onnx"));
    if (!fs::exists(model_path)) {
      ROS_FATAL("No ONNX model found in %s, and fallback '%s' doesn't exist.",
                config_dir.c_str(), model_path.c_str());
      return 1;
    }
    ROS_WARN("Using fallback model_path: %s", model_path.c_str());
  }




  // std::string model; nh.param("model_path",model,std::string("/home/spring/exchange/louis_dockers/rl_ws/src/onnx_inference_cpp/src/models/v3/policy.onnx"));
  sess.reset(new Ort::Session(env,model_path.c_str(),Ort::SessionOptions{}));

  in_names .push_back(sess->GetInputNameAllocated (0,Ort::AllocatorWithDefaultOptions()).release());
  out_names.push_back(sess->GetOutputNameAllocated(0,Ort::AllocatorWithDefaultOptions()).release());

  // pub_group_cmd  = nh.advertise<std_msgs::Float64MultiArray>("/rl_direct_controller/command",1); // rosrun topic_tools throttle messages /reach_policy/command 10.0 /rl_direct_controller/command
  pub_traj_cmd  = nh.advertise<trajectory_msgs::JointTrajectory>("/left_leg_controller/command",1);

  pub_sub_cmd = nh.advertise<sensor_msgs::JointState>("/subscriber_controller/desired_state",1); // ORIGINAL SUBSCRIBER CONTROLLER
  // pub_ruckig_goal = nh.advertise<std_msgs::Float64MultiArray>("/rl_ruckig_layer/command", 1);
  
  pub_last = nh.advertise<std_msgs::Float64MultiArray>("/reach_policy/last_action",1);
  ros::Subscriber sub = nh.subscribe("/reach_policy/obs",1,obsCB);

  ros::Timer timer = nh.createTimer(ros::Duration(0.02), timerCallback);

  ros::spin();
}
