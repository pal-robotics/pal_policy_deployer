#include "pal_policy_inferer/kang_mjlab_policy.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace pal_policy_inferer
{

const std::vector<std::string> excluded_action_joints = {
  "leg_left_femur_joint", "leg_left_knee_joint",
  "leg_right_femur_joint", "leg_right_knee_joint"
};

std::vector<double> MotionState::getCommand() const
{
  std::vector<double> cmd;
  cmd.reserve(num_joints * 2);
  cmd.insert(cmd.end(), joint_pos.begin(), joint_pos.end());
  cmd.insert(cmd.end(), joint_vel.begin(), joint_vel.end());
  return cmd;
}

void MotionState::updateFromOutputs(
  const float * out_joint_pos, int nj,
  const float * out_joint_vel, int /*nj2*/)
{
  for (int i = 0; i < nj && i < num_joints; ++i) {
    joint_pos[i] = out_joint_pos[i];
    joint_vel[i] = out_joint_vel[i];
  }
}

std::vector<std::string> splitString(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

std::vector<double> parseDoubles(const std::string & str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    values.push_back(std::stod(token));
  }
  return values;
}

bool isExcludedJoint(const std::string & joint_name)
{
  return std::find(excluded_action_joints.begin(), excluded_action_joints.end(), joint_name)
         != excluded_action_joints.end();
}

int indexVector(const std::vector<std::string> & container, const std::string & key)
{
  const auto it = std::find(container.cbegin(), container.cend(), key);
  if (it == container.cend()) {
    throw std::runtime_error("Unable to find the key : " + key);
  }
  return std::distance(container.cbegin(), it);
}

int getObservationTermDim(
  const PolicyConfig & cfg, const MotionState & mot, const std::string & term_name)
{
  if (term_name == "base_ang_vel") {return 3;}
  if (term_name == "base_lin_vel") {return 3;}
  if (term_name == "base_lin_acc") {return 3;}
  if (term_name == "imu_projected_gravity") {return 3;}
  if (term_name == "joint_pos") {return static_cast<int>(cfg.joint_names.size());}
  if (term_name == "joint_vel") {return static_cast<int>(cfg.joint_names.size());}
  if (term_name == "actions") {return static_cast<int>(cfg.action_joint_names.size());}
  if (term_name == "command") {
    return (cfg.task == "imitation") ? mot.commandDim() : 3;
  }
  return -1;
}

}  // namespace pal_policy_inferer
