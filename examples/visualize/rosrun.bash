echo "Enter index (0-) of the desired x-coordinates: "
read x_index
echo "Enter index (0-) of the desired y-coordinates: "
read y_index
echo "(Optional) Enter index (0-) of the desired z-coordinates. If no z-coordinates are desired, press return: "
read z_index

echo $x_index $y_index $z_index > "coordinate_indices.txt"
colcon build --packages-select visualize
source install/local_setup.bash
ros2 run visualize ros_visualize
# ros2 run rviz2 rviz2