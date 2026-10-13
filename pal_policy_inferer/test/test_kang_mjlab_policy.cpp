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
