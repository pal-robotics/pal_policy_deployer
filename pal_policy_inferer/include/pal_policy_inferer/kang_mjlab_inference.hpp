#ifndef PAL_POLICY_INFERER__KANG_MJLAB_INFERENCE_HPP_
#define PAL_POLICY_INFERER__KANG_MJLAB_INFERENCE_HPP_

// ONNX Runtime side of kang_mjlab_node. onnxruntime_cxx_api.h is deliberately kept out of
// this header (pimpl): combining it with rclcpp/rclcpp_action in one translation unit pushed
// the node's compile above 2 GB RAM and broke the debian build.

#include <memory>
#include <string>
#include <vector>

#include "pal_policy_inferer/kang_mjlab_policy.hpp"

namespace pal_policy_inferer
{

class KangMjlabModel
{
public:
  /// Raw output pointers of the last run(); valid until the next run() or destruction.
  /// joint_pos / joint_vel are null when the model has no such output.
  struct Outputs
  {
    const float * actions = nullptr;
    const float * joint_pos = nullptr;
    const float * joint_vel = nullptr;
  };

  KangMjlabModel();
  ~KangMjlabModel();
  KangMjlabModel(KangMjlabModel &&) noexcept;
  KangMjlabModel & operator=(KangMjlabModel &&) noexcept;

  /// @return path of the first regular *.onnx file in dir.
  /// @throws std::runtime_error if dir does not exist or contains no .onnx file.
  static std::string findOnnxModel(const std::string & dir);

  /// Create the inference session and fill cfg from the model: metadata (parsePolicyMetadata),
  /// num_inputs/num_outputs, obs_dim, action_dim and output indices. For non-imitation tasks
  /// also computeHistoryLength (imitation defers it until motion.num_joints is known).
  /// cfg.task must be set by the caller.
  /// @return one human-readable line per output and input, e.g. "Output 0: 'actions' [1,14]".
  /// @throws std::runtime_error on missing metadata or an obs_dim/history mismatch;
  ///         Ort::Exception (not a std::runtime_error) on ONNX Runtime failures.
  std::vector<std::string> load(
    const std::string & model_path, PolicyConfig & cfg, const MotionState & motion);

  /// Run one inference step. obs must have cfg.obs_dim elements. time_step is fed as the
  /// second input only when with_time_step is true (imitation models).
  /// @pre load() succeeded.
  Outputs run(std::vector<float> & obs, float time_step, bool with_time_step);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pal_policy_inferer

#endif  // PAL_POLICY_INFERER__KANG_MJLAB_INFERENCE_HPP_
