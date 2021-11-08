#!/bin/bash
# Summary: creates tmux session to that runs a demo session for the tecna_turtle robot
# TIP: :session-kill ends the session


# Define session name
SESSION="tecna_turtle-demo" # define session name


# Change working directory to project directory
WD=$(dirname $(realpath $0)) # this script's folder
WD=$(realpath ${WD}/..)
echo "session working dir: $WD"
cd ${WD}


# check if the tmux session already exists
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)
if [ ! "$SESSIONEXISTS" = "" ]
then
  #echo "tmux session ($SESSION) already exists!!!"
  #echo "  SESSIONEXISTS = $SESSIONEXISTS"
  #read -p "kill previous session? (y/n): " REPLY
  #if [ "$REPLY" = "y" ]
  #then
  #  echo "killing previous session (tmux kill-session -t $SESSION)"
  #  tmux kill-session -t $SESSION
  #  sleep 1
  #else
  #  echo "you can kill the session using: tmux kill-session -t $SESSION"
  #  exit 1.5
  #fi
  #
  tmux kill-session -t $SESSION
  ORANGE="\033[0;33m"
  NOCOLOR="\033[0m"
  echo -e $ORANGE
  echo "tmux session ($SESSION) already exists!!!"
  echo "  SESSIONEXISTS = $SESSIONEXISTS"
  echo "tmux session ($SESSION) killed"
  echo -e $NOCOLOR
  exit 0
fi


# setup env function
setup_env ()
{
  tmux send-keys -t $1 """source devel/setup.bash""" C-m
}


# Create session
echo "starting ${SESSION} tmux session.."
tmux new-session -d -s ${SESSION} -n gazebo # create session
STATUS=$?
# check if the tmux session already exists
if [ ! ${STATUS} -eq 0 ] ; then
  echo "WTF??? tmux session ($SESSION) already exists!!!"
  exit 1
fi
#
tmux set status on
tmux set mouse on


# Create tabs

# gazebo
# create windows and panes
#tmux new-window -t ${SESSION} -n gazebo # window already created with the session
setup_env ${SESSION}:gazebo.0
# run commands
tmux send-keys -t ${SESSION}:gazebo.0 """docker stop tecna_turtle""" C-m
tmux send-keys -t ${SESSION}:gazebo.0 """make docker_run""" C-m
tmux send-keys -t ${SESSION}:gazebo.0 """catkin_make""" C-m
tmux send-keys -t ${SESSION}:gazebo.0 """source devel/setup.bash""" C-m
tmux send-keys -t ${SESSION}:gazebo.0 """roslaunch tecna_turtle_gazebo spawn.launch \
    world_file:=\$(rospack find tecna_turtle_gazebo)/worlds/simple.world \
    gui:=false""" C-m
sleep 5

# teleop
# create windows and panes
tmux new-window -t ${SESSION} -n teleop
tmux split-window -t $SESSION:teleop.0 -v # split window (creates two panes)
setup_env ${SESSION}:teleop.0
setup_env ${SESSION}:teleop.1
# run commands
tmux send-keys -t ${SESSION}:teleop.0 """make docker_connect""" C-m
tmux send-keys -t ${SESSION}:teleop.0 """source devel/setup.bash""" C-m
tmux send-keys -t ${SESSION}:teleop.0 """roslaunch tecna_turtle_teleop keyboard.launch""" C-m
tmux send-keys -t ${SESSION}:teleop.1 """make docker_connect""" C-m
tmux send-keys -t ${SESSION}:teleop.1 """source devel/setup.bash""" C-m
tmux send-keys -t ${SESSION}:teleop.1 """rosrun map_server map_saver \
    -f \$(rospack find tecna_turtle_localization)/maps/simple_world""" # C-m
# select pane
tmux select-pane -t $SESSION:teleop.0

# slam (gmapping)
# create windows and panes
tmux new-window -t ${SESSION} -n slam
setup_env ${SESSION}:slam.0
# run commands
tmux send-keys -t ${SESSION}:slam.0 """make docker_connect""" C-m
tmux send-keys -t ${SESSION}:slam.0 """source devel/setup.bash""" C-m
tmux send-keys -t ${SESSION}:slam.0 """roslaunch tecna_turtle_localization lidar_gmapping.launch""" C-m

# rviz
# create windows and panes
tmux new-window -t ${SESSION} -n rviz
setup_env ${SESSION}:rviz.0
# run commands
tmux send-keys -t ${SESSION}:rviz.0 """make docker_connect""" C-m
tmux send-keys -t ${SESSION}:rviz.0 """source devel/setup.bash""" C-m
tmux send-keys -t ${SESSION}:rviz.0 """rosrun rviz rviz  \
  -d \$(rospack find tecna_turtle_localization)/rviz/lidar_gmapping.rviz""" C-m


# Attach to the tmux session
tmux attach -t $SESSION:teleop.0

