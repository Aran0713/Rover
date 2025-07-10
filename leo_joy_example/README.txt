What this package does:
- Outputs joystick commands to the `/cmd_vel` topic.
- Outputs linear.x and angular.z values changing as you move the rover

# How to use this package:
1. ssh into the rover (ssh -X pi@10.0.0.1)
2. In terminal 1:
  cd ~/ros_ws
  colcon build --packages-select leo_joy_example
  source install/setup.bash
  ros2 launch leo_joy_example joy_mapping.launch.py
3. In terminal 2:
  ssh into the rover
  ros2 topic echo /cmd_vel
4. Move the joystick and observe the output in terminal 2.

