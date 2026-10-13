#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/parameter.hpp>

namespace pal_policy_inferer
{

/// Build the PID gain parameters of the impedance controller from a policy's ONNX metadata.
///
/// The metadata `joint_stiffness` / `joint_damping` arrays are ordered like the policy's action
/// joints (i.e. `joint_names` minus the excluded mimic joints), so they map index-by-index.
///
/// @param joint_names  action joint names, in metadata order
/// @param stiffness    per-joint stiffness, written to `gains.<joint>.p`
/// @param damping      per-joint damping, written to `gains.<joint>.d`
/// @return parameters ordered as {<j0>.p, <j0>.d, <j1>.p, <j1>.d, ...}
/// @throws std::invalid_argument if stiffness/damping are missing or their size differs from
///         joint_names (a partial update would leave the controller with mixed gains).
inline std::vector<rclcpp::Parameter> buildImpedanceGainParameters(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & stiffness,
  const std::vector<double> & damping)
{
  if (stiffness.size() != joint_names.size() || damping.size() != joint_names.size()) {
    throw std::invalid_argument(
            "joint_stiffness (" + std::to_string(stiffness.size()) + ") / joint_damping (" +
            std::to_string(damping.size()) + ") size does not match the number of action joints (" +
            std::to_string(joint_names.size()) + ")");
  }
  std::vector<rclcpp::Parameter> params;
  params.reserve(joint_names.size() * 2);
  for (size_t i = 0; i < joint_names.size(); ++i) {
    params.emplace_back("gains." + joint_names[i] + ".p", stiffness[i]);
    params.emplace_back("gains." + joint_names[i] + ".d", damping[i]);
  }
  return params;
}

}  // namespace pal_policy_inferer
