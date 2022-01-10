
tecna_turtle is a ROS project that allows to guide a Tecna robot (with differential wheel drive) to follow user defined trajectories


# WARNING:

If you are using an nvidia video card you must use `FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04` in your Dockerfile
and run docker with `--gpus all`


# Setup

    make docker_create_image

# Run

    # Start simulator
    make docker_run
    catkin_make  # build the project
    source devel/setup.bash
    roslaunch tecna_turtle_gazebo spawn.launch world_file:=$(rospack find tecna_turtle_gazebo)/worlds/simple.world gui:=false

    # Run visualization
    make docker_connect
    source devel/setup.bash
    rviz -d src/tecna_turtle_description/rviz/tecna_turtle.rviz

    # Teleoperation (command line)
    make docker_connect
    source devel/setup.bash
    rostopic pub -r 10 /cmd_vel \
      geometry_msgs/Twist '{linear: [ .5, 0, 0 ], angular: [0, 0, .3]}'

    # Teleoperation (keyboard)
    make docker_connect
    source devel/setup.bash
    roslaunch tecna_turtle_teleop keyboard.launch

-------

# In progress

    demos/dbg.bash


