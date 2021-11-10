cmake_minimum_required(VERSION 2.8.3)
project(tecna_turtle_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(DIRECTORY msg FILES DiffDriveWheelVel.msg)
generate_messages(DEPENDENCIES std_msgs)

