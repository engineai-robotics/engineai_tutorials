"""Optional source-side rosbag capture. Never publishes motion or goal commands."""
import argparse
from pathlib import Path
import subprocess

TOPICS = ('/mapping_scan', '/scan', '/odom', '/Odometry', '/tf', '/tf_static',
          '/map', '/navigation_map', '/pm01/slam_pose', '/pm01/slam_match_update',
          '/pm01/slam_map_revision', '/pm01/slam_graph_anchors', '/pm01/localization_status',
          '/pm01/map_update_status', '/pm01/ground_plane_status', '/lio/status',
          '/pm01/navigation/motion_status')
CLOUD_TOPICS = ('/livox/lidar', '/cloud_registered_body', '/livox/points')
NAVIGATION_TOPICS = ('/cmd_vel_nav', '/navigation_cmd_vel_requested', '/cmd_vel', '/motion/body_vel_cmd',
                     '/pm01/navigation_readiness', '/pm01/navigation_wait_status',
                     '/motion/motion_state', '/plan', '/local_plan',
                     '/navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/status')


def command(output, include_clouds=False, include_navigation=False):
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError('Use a new recording directory')
    return ['ros2', 'bag', 'record', '--output', str(output),
            *TOPICS, *(CLOUD_TOPICS if include_clouds else ()),
            *(NAVIGATION_TOPICS if include_navigation else ())]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--include-clouds', action='store_true')
    parser.add_argument('--include-navigation', action='store_true',
                        help='Passively record path, goal feedback and velocity commands')
    args = parser.parse_args()
    return subprocess.call(command(args.output, args.include_clouds, args.include_navigation))
