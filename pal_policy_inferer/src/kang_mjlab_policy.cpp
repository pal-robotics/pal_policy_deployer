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

void parsePolicyMetadata(
  const std::map<std::string, std::string> & metadata, PolicyConfig & cfg)
{
  auto require = [&](const std::string & key) -> const std::string & {
      const auto it = metadata.find(key);
      if (it == metadata.end()) {
        throw std::runtime_error("Missing '" + key + "'");
      }
      return it->second;
    };
  auto optional = [&](const std::string & key) {
      const auto it = metadata.find(key);
      return it == metadata.end() ? std::vector<double>{} : parseDoubles(it->second);
    };

  if (const auto it = metadata.find("run_path"); it != metadata.end()) {
    cfg.run_path = it->second;
  }
  cfg.joint_names = splitString(require("joint_names"), ',');
  cfg.default_joint_pos = parseDoubles(require("default_joint_pos"));
  cfg.action_scale = parseDoubles(require("action_scale"));
  cfg.observation_names = splitString(require("observation_names"), ',');
  cfg.joint_stiffness = optional("joint_stiffness");
  cfg.joint_damping = optional("joint_damping");

  cfg.action_joint_names.clear();
  cfg.action_offset.clear();
  for (size_t i = 0; i < cfg.joint_names.size(); ++i) {
    if (!isExcludedJoint(cfg.joint_names[i])) {
      cfg.action_joint_names.push_back(cfg.joint_names[i]);
      cfg.action_offset.push_back(cfg.default_joint_pos[i]);
    }
  }
}

bool computeHistoryLength(PolicyConfig & cfg, const MotionState & mot)
{
  cfg.single_obs_dim = 0;
  for (const auto & term_name : cfg.observation_names) {
    cfg.single_obs_dim += getObservationTermDim(cfg, mot, term_name);
  }
  if (cfg.single_obs_dim <= 0 || cfg.obs_dim % cfg.single_obs_dim != 0) {
    return false;
  }
  cfg.history_length = cfg.obs_dim / cfg.single_obs_dim;
  return true;
}

}  // namespace pal_policy_inferer
