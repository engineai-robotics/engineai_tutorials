#include <gtest/gtest.h>
#include "pm01_navigation_nav2/quiet_window.hpp"
using pm01_fastlio_nav::QuietWindow;

TEST(StablePoseWindow, RequiresDistinctStablePosesAndRejectsCoasting) {
  pm01_fastlio_nav::StablePoseWindow poses;
  const auto sample = [&](double t, double x, double a) { return poses.update(t,x,0.,a,.05,.1); };
  EXPECT_FALSE(sample(1.,0.,3.14));
  EXPECT_FALSE(sample(1.2,0.002,-3.14));
  EXPECT_FALSE(sample(1.2,0.002,-3.14));
  EXPECT_TRUE(sample(1.41,0.001,3.14));
  EXPECT_FALSE(sample(1.5,.03,3.14));
  EXPECT_FALSE(sample(1.6,0.,3.14));
  EXPECT_FALSE(sample(1.9,0.,3.14));
  EXPECT_FALSE(sample(2.5,0.,3.14));
}

TEST(QuietWindow, DoesNotAcceptASingleZeroCrossing) {
  QuietWindow window;
  EXPECT_FALSE(window.update(1., true));
  EXPECT_FALSE(window.update(1.2, true));
  EXPECT_FALSE(window.update(1.3, false));
  EXPECT_FALSE(window.update(1.5, true));
  EXPECT_TRUE(window.update(2., true));
}
TEST(QuietWindow, PoseLeavingWindowRestartsSettling) {
  QuietWindow window;
  EXPECT_FALSE(window.update(1., true));
  EXPECT_FALSE(window.update(1.3, false));
  EXPECT_FALSE(window.update(2., true));
  EXPECT_FALSE(window.update(2.2, true));
  EXPECT_TRUE(window.update(2.5, true));
  window.reset();
  EXPECT_FALSE(window.update(3., true));
}
TEST(QuietWindow, ClockRewindCannotComplete) {
  QuietWindow window;
  EXPECT_FALSE(window.update(2., true));
  EXPECT_FALSE(window.update(1., true));
  EXPECT_FALSE(window.update(1.5, true));
  EXPECT_TRUE(window.update(2., true));
}
