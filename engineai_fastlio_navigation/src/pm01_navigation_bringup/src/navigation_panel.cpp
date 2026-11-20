// SPDX-License-Identifier: Apache-2.0
#include <nav2_rviz_plugins/nav2_panel.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pm01_navigation_bringup {
class NavigationPanel final : public nav2_rviz_plugins::Nav2Panel {};
}

PLUGINLIB_EXPORT_CLASS(pm01_navigation_bringup::NavigationPanel, rviz_common::Panel)
