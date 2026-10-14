#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "pal_policy_inferer/kang_mjlab_inference.hpp"

using namespace pal_policy_inferer;
namespace fs = std::filesystem;

TEST(KangMjlabModel, FindOnnxModel)
{
  const fs::path dir = fs::temp_directory_path() / "kang_mjlab_find_onnx_test";
  fs::remove_all(dir);
  fs::create_directories(dir);
  std::ofstream(dir / "notes.txt") << "x";
  EXPECT_THROW(KangMjlabModel::findOnnxModel(dir.string()), std::runtime_error);
  std::ofstream(dir / "policy.onnx") << "x";
  EXPECT_EQ(KangMjlabModel::findOnnxModel(dir.string()), (dir / "policy.onnx").string());
  EXPECT_THROW(KangMjlabModel::findOnnxModel((dir / "missing").string()), std::runtime_error);
  fs::remove_all(dir);
}

// The shipped model lives in the sibling pal_policy_deployer package; CMake only defines
// the path when that source tree is present (it is not in a standalone debian build).
TEST(KangMjlabModel, LoadAndRunImitationModel)
{
#ifndef KANG_TEST_IMITATION_MODEL_DIR
  GTEST_SKIP() << "shipped imitation model not available";
#else
  const std::string path = KangMjlabModel::findOnnxModel(KANG_TEST_IMITATION_MODEL_DIR);

  PolicyConfig cfg;
  cfg.task = "imitation";
  MotionState motion;
  KangMjlabModel model;
  const auto io = model.load(path, cfg, motion);

  EXPECT_EQ(cfg.num_inputs, 2);
  EXPECT_EQ(cfg.num_outputs, 7);
  EXPECT_EQ(io.size(), 9u);  // one line per output + input
  EXPECT_EQ(cfg.obs_dim, 95);
  EXPECT_EQ(cfg.action_dim, 14);
  EXPECT_EQ(cfg.out_idx_actions, 0);
  EXPECT_EQ(cfg.out_idx_joint_pos, 1);
  EXPECT_EQ(cfg.out_idx_joint_vel, 2);
  EXPECT_EQ(cfg.joint_names.size(), 18u);
  EXPECT_EQ(cfg.history_length, 0);  // deferred for imitation until num_joints is known

  std::vector<float> obs(cfg.obs_dim, 0.0f);
  const auto out = model.run(obs, 0.0f, true);
  ASSERT_NE(out.actions, nullptr);
  ASSERT_NE(out.joint_pos, nullptr);
  ASSERT_NE(out.joint_vel, nullptr);
#endif
}
