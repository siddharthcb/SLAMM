# SLAMM
SLAM-Project

A SLAM based 2D Autonomous Navigation System for Robots with integrated onboard range sensors

Package Pre-requisites: linux Ubuntu 18.04 ROS - version="melodic"

Package Description: This package consists of sub-packages integrated to successfully perform a fully autonomous navigation stack.

clone this to your catkin workspace. run these 3 commands separetely to create a map 1)roslaunch ydlidar_ros TG.launch 2)roslaunch hector_slam_launch tutorial.launch 3)rviz rviz

Once you have a complete map or something acceptable, you can save it to use it later in the navigation stack. To save it, use the following command: rosrun map_server map_saver -f your_mapname

navigate to navigation/run and paste your saved map files. navigate to navigation/run/launch/move_base.launch and change the map name accordingly.

Run 1. roslaunch run move_base.launch 2. open rviz rviz for slam navigation

navigate to sensors/rplidar_ros/scripts/uptime.py: This is a python script that keeps reading the cmd_vel messages generated by move_base in a loop and converts the messages to appropriate rpml and rpmr. These values are sent to the moab board that controlls the motor rpm. Run this python script after /cmd_vel are generated. -Calibrate the speed factors in this script according to different motor setups

Package Usage Guide:

navigation/run: This sub_package governs all the commands, launch files, and parameters that decide the navigation performance -Goto launch directory inside this sub_package. -Calibrate the parameter values in base_local_planner_params.yaml, global_costmap_params.yaml, costmap_common_params.yaml, local_costmap_params.yaml to tune the navigation stack according to your bot and operating environment.

During calibration take feedback after running and keep calibrating the paramater values in the .yaml files mentioned above.

amcl: Subscribes to scan messages (/scan), map data(/map) and reads transform messages from laser to base frame and base to odom. Determines the robots pose and publishes (/amcl_pose) and tf messages from odom to map frame.

move_base: Subscribes to the pose data, scan messages, the goal pose, and map data. Integrates global_planner, navfn, base_local_planner, clear_costmap_recovery, costmap_2d, dwa_local_planner, nav_core, move_slow_and_clear, voxel_grid, rotate_recovery Generates costmap, global path and local path and publishes the same.
# SLAM
# SLAM
