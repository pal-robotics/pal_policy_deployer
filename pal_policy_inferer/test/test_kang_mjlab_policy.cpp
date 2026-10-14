#include <gtest/gtest.h>

#include "pal_policy_inferer/kang_mjlab_policy.hpp"

using namespace pal_policy_inferer;

TEST(KangMjlabPolicy, SplitAndParse)
{
  EXPECT_EQ(splitString("a,b,c", ','), (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_EQ(parseDoubles("1.5,-2,0"), (std::vector<double>{1.5, -2.0, 0.0}));
  EXPECT_THROW(parseDoubles("1,x"), std::invalid_argument);
}

TEST(KangMjlabPolicy, ExcludedJointsAndIndex)
{
  EXPECT_TRUE(isExcludedJoint("leg_left_knee_joint"));
  EXPECT_FALSE(isExcludedJoint("leg_left_hip_z_joint"));
  EXPECT_EQ(indexVector({"a", "b", "c"}, "c"), 2);
  EXPECT_THROW(indexVector({"a"}, "z"), std::runtime_error);
}

TEST(KangMjlabPolicy, MotionState)
{
  MotionState m;
  m.num_joints = 2;
  m.joint_pos = {0.0, 0.0};
  m.joint_vel = {0.0, 0.0};
  const float pos[] = {1.0f, 2.0f};
  const float vel[] = {3.0f, 4.0f};
  m.updateFromOutputs(pos, 2, vel, 2);
  EXPECT_EQ(m.commandDim(), 4);
  EXPECT_EQ(m.getCommand(), (std::vector<double>{1.0, 2.0, 3.0, 4.0}));
}

TEST(KangMjlabPolicy, ObservationTermDim)
{
  PolicyConfig cfg;
  cfg.joint_names = {"j1", "j2", "j3"};
  cfg.action_joint_names = {"j1", "j2"};
  MotionState m;
  m.num_joints = 3;

  cfg.task = "loco";
  EXPECT_EQ(getObservationTermDim(cfg, m, "imu_projected_gravity"), 3);
  EXPECT_EQ(getObservationTermDim(cfg, m, "joint_pos"), 3);
  EXPECT_EQ(getObservationTermDim(cfg, m, "actions"), 2);
  EXPECT_EQ(getObservationTermDim(cfg, m, "command"), 3);
  EXPECT_EQ(getObservationTermDim(cfg, m, "bogus"), -1);

  cfg.task = "imitation";
  EXPECT_EQ(getObservationTermDim(cfg, m, "command"), 6);
}

TEST(KangMjlabPolicy, ParsePolicyMetadata)
{
  std::map<std::string, std::string> md = {
    {"run_path", "runs/x"},
    {"joint_names", "hip,leg_left_knee_joint,ankle"},
    {"default_joint_pos", "0.1,0.2,0.3"},
    {"action_scale", "0.5,0.5"},
    {"observation_names", "joint_pos,actions"},
    {"joint_stiffness", "10,20"},
  };
  PolicyConfig cfg;
  parsePolicyMetadata(md, cfg);
  EXPECT_EQ(cfg.run_path, "runs/x");
  EXPECT_EQ(cfg.joint_names.size(), 3u);
  EXPECT_EQ(cfg.observation_names, (std::vector<std::string>{"joint_pos", "actions"}));
  EXPECT_EQ(cfg.joint_stiffness, (std::vector<double>{10.0, 20.0}));
  EXPECT_TRUE(cfg.joint_damping.empty());
  // Excluded (mimic) joints are dropped from the action set together with their offset.
  EXPECT_EQ(cfg.action_joint_names, (std::vector<std::string>{"hip", "ankle"}));
  EXPECT_EQ(cfg.action_offset, (std::vector<double>{0.1, 0.3}));

  md.erase("action_scale");
  try {
    parsePolicyMetadata(md, cfg);
    FAIL() << "expected throw";
  } catch (const std::runtime_error & e) {
    EXPECT_STREQ(e.what(), "Missing 'action_scale'");
  }
}

TEST(KangMjlabPolicy, ComputeHistoryLength)
{
  PolicyConfig cfg;
  cfg.task = "loco";
  cfg.joint_names = {"j1", "j2"};
  cfg.action_joint_names = {"j1", "j2"};
  cfg.observation_names = {"joint_pos", "command"};  // 2 + 3
  MotionState m;

  cfg.obs_dim = 15;
  EXPECT_TRUE(computeHistoryLength(cfg, m));
  EXPECT_EQ(cfg.single_obs_dim, 5);
  EXPECT_EQ(cfg.history_length, 3);

  cfg.obs_dim = 16;
  EXPECT_FALSE(computeHistoryLength(cfg, m));

  cfg.task = "imitation";  // command dim = 2 * num_joints
  m.num_joints = 2;
  cfg.obs_dim = 12;
  EXPECT_TRUE(computeHistoryLength(cfg, m));
  EXPECT_EQ(cfg.single_obs_dim, 6);
  EXPECT_EQ(cfg.history_length, 2);
}
