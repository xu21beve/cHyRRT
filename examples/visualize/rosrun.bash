colcon build --packages-select visualize
source install/local_setup.bash
ros2 run visualize ros_visualize
ros2 run rviz2 rviz2