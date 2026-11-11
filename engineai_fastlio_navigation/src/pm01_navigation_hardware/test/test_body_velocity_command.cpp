#include <array>
#include <limits>
#include <gtest/gtest.h>
#include "pm01_navigation_hardware/body_velocity_command.hpp"

TEST(BodyVelocityCommand, OptionalClampRetainsExistingBehaviour) {
  std::array<double, 3> out{};
  ASSERT_TRUE(pm01_fastlio_nav::convert_body_velocity({1.2, -.6, 1.}, {.7, .4, .7}, true, out));
  EXPECT_DOUBLE_EQ(out[0], .7);
  EXPECT_DOUBLE_EQ(out[1], -.4);
  EXPECT_DOUBLE_EQ(out[2], .7);
}

TEST(BodyVelocityCommand, PassthroughPreservesFiniteCommandsAboveOptionalLimits) {
  const std::array<double, 3> in{1.2, -.6, 1.};
  std::array<double, 3> out{};
  ASSERT_TRUE(pm01_fastlio_nav::convert_body_velocity(in, {.7, .4, .7}, false, out));
  EXPECT_EQ(in, out);
}

TEST(BodyVelocityCommand, PassthroughPreservesUnitsSignsAndZero) {
  const std::array<double, 3> in{.31, -.12, 0.};
  std::array<double, 3> out{};
  ASSERT_TRUE(pm01_fastlio_nav::convert_body_velocity(in, {.7, .4, .7}, false, out));
  EXPECT_EQ(in, out);
}

TEST(BodyVelocityCommand, NonFiniteInputNeverBecomesAnExecutableCommand) {
  for (const bool limited : {true, false}) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity()}) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        std::array<double, 3> in{.3, .1, .2};
        std::array<double, 3> out{};
        in[axis] = invalid;
        EXPECT_FALSE(pm01_fastlio_nav::convert_body_velocity(in, {.7, .4, .7}, limited, out));
        EXPECT_EQ(out, (std::array<double, 3>{0., 0., 0.}));
      }
    }
  }
}
