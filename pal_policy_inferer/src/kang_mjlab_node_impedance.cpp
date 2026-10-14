#include "kang_mjlab_node.hpp"

#include <pal_policy_inferer/impedance_gains.hpp>

#include <chrono>
#include <functional>

// The parameter client templates are kept apart from the rest of the node to limit the peak
// compiler memory per translation unit.

namespace pal_policy_inferer
{

static const std::string IMPEDANCE_CONTROLLER_NODE = "/rl_impedance_controller";

void KangMjlabNode::createImpedanceGainsInterfaces()
{
  impedance_params_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(this, IMPEDANCE_CONTROLLER_NODE);
  // Gain updates are sent from this timer so that all parameter-client calls happen in the
  // spin thread (the switch action runs in a detached thread) and so that startup works even
  // if the controller is spawned after this node.
  impedance_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&KangMjlabNode::impedanceTimerCb, this));
}

void KangMjlabNode::impedanceTimerCb()
{
  if (pending_gains_idx_.load() < 0) {return;}
  if (!impedance_params_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 5000,
      "Waiting for %s parameter services to set the impedance gains...",
      IMPEDANCE_CONTROLLER_NODE.c_str());
    return;
  }
  applyImpedanceGains(pending_gains_idx_.exchange(-1));
}

void KangMjlabNode::applyImpedanceGains(int idx)
{
  const PolicyInstance & p = policies_[idx];
  std::vector<rclcpp::Parameter> params;
  try {
    params = buildImpedanceGainParameters(
      p.config.action_joint_names, p.config.joint_stiffness, p.config.joint_damping);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(
      get_logger(), "[Policy '%s'] Not setting impedance gains: %s", p.name.c_str(), e.what());
    return;
  }
  std::vector<std::string> names;
  for (const auto & param : params) {names.push_back(param.get_name());}

  const std::string policy_name = p.name;
  impedance_params_client_->get_parameters(
    names,
    [this, params, policy_name](std::shared_future<std::vector<rclcpp::Parameter>> old_future) {
      const auto old_params = old_future.get();
      impedance_params_client_->set_parameters(
        params,
        [this, params, old_params, policy_name](
          std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
          const auto results = future.get();
          size_t changed = 0, failed = 0;
          for (size_t i = 0; i < params.size(); ++i) {
            const bool known_old = i < old_params.size();
            // Unknown parameters come back as PARAMETER_NOT_SET instead of a double.
            const std::string old_value =
              (known_old &&
              old_params[i].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) ?
              std::to_string(old_params[i].as_double()) : "?";
            if (i < results.size() && results[i].successful) {
              // Only log gains that actually change; the summary below covers the rest.
              if (known_old && !gainChanged(old_params[i], params[i])) {continue;}
              ++changed;
              RCLCPP_INFO(
                get_logger(), "[Policy '%s'] %s: %s -> %f",
                policy_name.c_str(), params[i].get_name().c_str(), old_value.c_str(), params[i].as_double());
            } else {
              ++failed;
              RCLCPP_ERROR(
                get_logger(), "[Policy '%s'] %s: FAILED to set %f (still %s): %s",
                policy_name.c_str(), params[i].get_name().c_str(), params[i].as_double(), old_value.c_str(),
                i < results.size() ? results[i].reason.c_str() : "no response");
            }
          }
          RCLCPP_INFO(
            get_logger(), "[Policy '%s'] Impedance gains of %s: %zu changed, %zu unchanged, %zu failed",
            policy_name.c_str(), IMPEDANCE_CONTROLLER_NODE.c_str(), changed,
            params.size() - changed - failed, failed);
        });
    });
}

}  // namespace pal_policy_inferer
