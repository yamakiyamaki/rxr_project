# rxr_project
## How to run
1. In a terminal, run this command. 
ros2 launch andino_gz andino_gz.launch.py nav2:=True
2. In Gazebo, push start button.
3. In Rviz, do "pose estimate".
4. In another terminal, run this command.
ros2 run path_planner_example path_planner_node --ros-args -p use_sim_time:=True
5. In Rviz, set Nav2 goal.
