#include <gtest/gtest.h>

#include <stdexcept>

#include "pal_policy_inferer/impedance_gains.hpp"

using pal_policy_inferer::buildImpedanceGainParameters;

TEST(ImpedanceGains, MapsStiffnessToPAndDampingToD)
{
  const auto params = buildImpedanceGainParameters(
    {"leg_left_1_joint", "leg_left_length_joint"}, {100.0, 1600.0}, {6.366, 101.859});

  ASSERT_EQ(params.size(), 4u);
  EXPECT_EQ(params[0].get_name(), "gains.leg_left_1_joint.p");
  EXPECT_DOUBLE_EQ(params[0].as_double(), 100.0);
  EXPECT_EQ(params[1].get_name(), "gains.leg_left_1_joint.d");
  EXPECT_DOUBLE_EQ(params[1].as_double(), 6.366);
  EXPECT_EQ(params[2].get_name(), "gains.leg_left_length_joint.p");
  EXPECT_DOUBLE_EQ(params[2].as_double(), 1600.0);
  EXPECT_EQ(params[3].get_name(), "gains.leg_left_length_joint.d");
  EXPECT_DOUBLE_EQ(params[3].as_double(), 101.859);
}

TEST(ImpedanceGains, ThrowsOnLengthMismatch)
{
  EXPECT_THROW(
    buildImpedanceGainParameters({"a_joint", "b_joint"}, {1.0}, {1.0, 2.0}),
    std::invalid_argument);
  EXPECT_THROW(
    buildImpedanceGainParameters({"a_joint"}, {1.0}, {}),
    std::invalid_argument);
}

TEST(ImpedanceGains, ThrowsWhenMetadataMissing)
{
  EXPECT_THROW(buildImpedanceGainParameters({"a_joint"}, {}, {}), std::invalid_argument);
}
