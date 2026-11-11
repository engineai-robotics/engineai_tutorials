#include <gtest/gtest.h>
#include "pm01_navigation_nav2/quiet_window.hpp"
using pm01_fastlio_nav::QuietWindow;

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
