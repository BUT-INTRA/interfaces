# Interfacing Simulator

This repository contains custom ROS interfaces and a simple simulator for BUT-INTRA interfacing testing. The interfaces and simulator are located in the `ricaip_interfaces` and `interfacing_simulator` packages, respectively. The simulator comprises two virtual robots moving around, each offering the interfaces defined in the `AI4DI_BUT-INTRA_interfacing.pdf` document. Please note the robots are not able to navigate to desired coordinates, their trajectories are pre-programmed. Feel free to modify the code to simulate all necessary robot states.

## Environment

The simulator was tested on Ubuntu 20.04 with ROS 2 Foxy Fitzroy (desktop version). Please follow standard installation and environment setting steps at https://docs.ros.org/en/foxy/.

## Workspace Building

Clone the repository and run build command `colcon build` in the `/workspace` directory. Do not forget to source the environment using `. install/setup.bash` after that.

## Testing

Run the simulator: `ros2 run interfacing_simulator simulator`

See the available topics / services / actions: `ros2 topic list` / `ros2 service list` / `ros2 action list`

See the data being published on a topic: `ros2 topic echo /factory/robot_1/pose`

Call a service: `TODO`

Call an action: `TODO`
