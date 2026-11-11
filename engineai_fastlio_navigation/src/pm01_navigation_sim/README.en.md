# pm01_navigation_sim

[简体中文](README.md) | English

Simulation platform adaptation. Converts walking commands and provides dynamic joint-chain TF and Gazebo GUI lifecycle management.

/cmd_vel → /pm01/walking_cmd_vel; LIO and joint TF → /odom.

During preparation, the simulation plugin holds the initial pose while the physics clock advances to activate the controllers. The initial-pose publisher waits for controller activation and completes its hold using simulation time. The walking policy starts after that publisher exits, so they do not write joint targets at the same time. The startup helper waits for the walking-policy service, releases the physics hold after a successful full standing reset, and confirms two consecutive simulation seconds of stable standing using new IMU data. Bringup then starts LIO. Preparation failures abort startup.

This package does not depend on hardware motion-control messages. Bringup owns the default world, gait configuration, and launch composition. Simulation navigation does not use Gazebo ground truth.
