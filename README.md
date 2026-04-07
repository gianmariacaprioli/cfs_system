# REMINDER:
## About this project (Custom Crazyswarm2)

This repository contains a custom version of the original **Crazyswarm2** project. 

Specific modifications have been made, particularly to the Python `crazyflie_server`, to implement certain behaviors and features originally available in ROS1 that are currently still in development or being ported to the ROS2 ecosystem.

**Credits Disclaimer:** I do not claim any credit for the work done by the original Crazyswarm2 developers. This custom version was created primarily as a personal programming exercise in ROS2/Python, and secondly as a practical tool to facilitate my ongoing research work. All core architecture credits belong to the original authors and contributors.


When cloning this repository, please use the following command to ensure all dependencies work correctly:
```bash
git clone https://github.com/gianmariacaprioli/cfs_system.git crazyswarm2
```

#README:

* Local CI: [![ROS 2](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/crazyswarm2/actions/workflows/ci-ros2.yml)
* Rolling Dev CI : [![Build Status](https://build.ros2.org/job/Rdev__crazyflie__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__crazyflie__ubuntu_noble_amd64/)
* Jazzy Dev CI: [![Build Status](https://build.ros2.org/job/Jdev__crazyswarm2__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__crazyswarm2__ubuntu_noble_amd64/)
* Humble Dev CI: [![Build Status](https://build.ros2.org/job/Hdev__crazyswarm2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__crazyswarm2__ubuntu_jammy_amd64/)


# Crazyswarm2
A ROS 2-based stack for Bitcraze Crazyflie multirotor robots.

The documentation is available here: https://imrclab.github.io/crazyswarm2/.

## Troubleshooting
Please start a [Discussion](https://github.com/IMRCLab/crazyswarm2/discussions) for...

- Getting Crazyswarm2 to work with your hardware setup.
- Advice on how to use it to achieve your goals.
- Rough ideas for a new feature.

Please open an [Issue](https://github.com/IMRCLab/crazyswarm2/issues) if you believe that fixing your problem will involve a **change in the Crazyswarm2 source code**, rather than your own configuration files. For example...

- Bug reports.
- New feature proposals with details.
