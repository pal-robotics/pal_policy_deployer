#ifndef PAL_POLICY_INFERER__KANG_MJLAB_POLICY_HPP_
#define PAL_POLICY_INFERER__KANG_MJLAB_POLICY_HPP_

// ROS- and ONNX-free policy types and helpers used by kang_mjlab_node.

#include <string>
#include <vector>

namespace pal_policy_inferer
{

/// Joints excluded from action outputs (mechanically constrained / mimic).
extern const std::vector<std::string> excluded_action_joints;

/// Autoregressive motion state for imitation policies.
struct MotionState
{
  std::vector<double> joint_pos;
  std::vector<double> joint_vel;
  int num_joints = 0;
  int timestep = 0;
  int frames = 0;
  bool loaded = false;

  /// @return joint_pos followed by joint_vel (size commandDim()).
  std::vector<double> getCommand() const;

  int commandDim() const {return num_joints * 2;}

  /// Copy the first min(nj, num_joints) policy outputs into joint_pos / joint_vel.
  void updateFromOutputs(
    const float * out_joint_pos, int nj,
    const float * out_joint_vel, int nj2);
};

/// Policy configuration parsed from ONNX custom metadata.
struct PolicyConfig
{
  std::string run_path;
  std::string task;  // "loco" or "imitation"
  std::vector<std::string> joint_names;
  std::vector<std::string> action_joint_names;
  std::vector<double> default_joint_pos;
  std::vector<double> action_offset;
  std::vector<double> action_scale;
  std::vector<double> joint_stiffness;
  std::vector<double> joint_damping;
  std::vector<std::string> observation_names;
  int obs_dim = 0;
  int action_dim = 0;
  int history_length = 0;
  int single_obs_dim = 0;
  int num_inputs = 0;
  int num_outputs = 0;
  int out_idx_actions = -1;
  int out_idx_joint_pos = -1;
  int out_idx_joint_vel = -1;
};

std::vector<std::string> splitString(const std::string & str, char delimiter);

/// Parse a comma-separated list of doubles.
/// @throws std::invalid_argument / std::out_of_range from std::stod on malformed tokens.
std::vector<double> parseDoubles(const std::string & str);

bool isExcludedJoint(const std::string & joint_name);

/// @return position of key in container.
/// @throws std::runtime_error if key is not present.
int indexVector(const std::vector<std::string> & container, const std::string & key);

/// @return dimension of a single (non-history) observation term, or -1 if unknown.
int getObservationTermDim(
  const PolicyConfig & cfg, const MotionState & mot, const std::string & term_name);

}  // namespace pal_policy_inferer

#endif  // PAL_POLICY_INFERER__KANG_MJLAB_POLICY_HPP_
