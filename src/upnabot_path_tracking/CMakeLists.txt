cmake_minimum_required(VERSION 2.8.3)
project(upnabot_path_tracking)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  #  roscpp
  rospy
  geometry_msgs nav_msgs std_msgs visualization_msgs
  actionlib
  dynamic_reconfigure
  #path_tracking  # nuhnuh: I seems that this should be added, but leads to error
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PursuitPathTracking.cfg
)


