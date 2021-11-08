cmake_minimum_required(VERSION 2.8.3)
project(path_tracking)

# This allows to import in nodes python packages from the src directory
# Tips: 
#  src/yujin_ocs/yocs_ar_pair_approach/CMakeLists.txt can be used as reference for more CMakeLists.txt instructions
#  http://www.artificialhumancompanions.com/structure-python-based-ros-package/ info related to python ros packages
catkin_python_setup()

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  #  dynamic_reconfigure
  actionlib message_generation geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  cfg/PathTracking.cfg
#)

# Add actions
add_action_files(DIRECTORY action FILES TrackPath.action)
# Generate messages
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)


