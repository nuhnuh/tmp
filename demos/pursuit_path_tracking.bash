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
  ORANGE="\033[0;33m"
  NOCOLOR="\033[0m"
  echo -e $ORANGE
  echo "tmux session ($SESSION) already exists!!!"
  echo "  SESSIONEXISTS = $SESSIONEXISTS"
  tmux kill-session -t $SESSION
  echo "tmux session ($SESSION) killed"
  echo "stopping docker tecna_turtle .."
  docker stop tecna_turtle -t 0
  echo ".. docker tecna_turtle stopped"
  echo -e $NOCOLOR
  exit 0
fi


# setup env function
setup_env ()
{
  tmux send-keys -t $1 """make docker_connect""" C-m
  tmux send-keys -t $1 """source devel/setup.bash""" C-m
}


# Create session
echo "starting ${SESSION} tmux session.."
tmux new-session -d -s ${SESSION} -n docker # create session
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

# docker run
#tmux send-keys -t ${SESSION}:docker.0 """docker stop tecna_turtle""" C-m
tmux send-keys -t ${SESSION}:docker.0 """make docker_run""" C-m
tmux send-keys -t ${SESSION}:docker.0 """#catkin_make""" C-m
tmux send-keys -t ${SESSION}:docker.0 """roscore""" C-m
sleep 3

# gazebo
# create windows and panes
tmux new-window -t ${SESSION} -n gazebo
setup_env ${SESSION}:gazebo.0
# run commands
tmux send-keys -t ${SESSION}:gazebo.0 """roslaunch tecna_turtle_gazebo spawn.launch \
    world_file:=\$(rospack find tecna_turtle_gazebo)/worlds/simple.world \
    gui:=false""" C-m

# amcl (localization)
# create windows and panes
tmux new-window -t ${SESSION} -n amcl
setup_env ${SESSION}:amcl.0
# run commands
tmux send-keys -t ${SESSION}:amcl.0 """roslaunch tecna_turtle_localization lidar_amcl.launch""" C-m

# path tracking
# create windows and panes
tmux new-window -t ${SESSION} -n path_tracking
tmux split-window -t ${SESSION}:path_tracking.0 -h # split window (creates two panes)
setup_env ${SESSION}:path_tracking.0
setup_env ${SESSION}:path_tracking.1
# run commands
tmux send-keys -t ${SESSION}:path_tracking.0 """roslaunch upnabot_path_tracking pursuit_path_tracking.launch""" C-m
tmux send-keys -t ${SESSION}:path_tracking.1 """#\$(rospack find upnabot_path_tracking)/scripts/track_paths_0.bash map""" C-m
tmux send-keys -t ${SESSION}:path_tracking.1 """\$(rospack find upnabot_path_tracking)/scripts/track_paths_0.bash odom""" C-m
# select pane
tmux select-pane -t $SESSION:path_tracking.1

# rqt
# create windows and panes
tmux new-window -t ${SESSION} -n rqt
tmux split-window -t ${SESSION}:rqt.0 -v # split window (creates two panes)
setup_env ${SESSION}:rqt.0
setup_env ${SESSION}:rqt.1
# run commands
tmux send-keys -t ${SESSION}:rqt.0 """rosrun rqt_reconfigure rqt_reconfigure""" # C-m
tmux send-keys -t ${SESSION}:rqt.1 """rosrun rqt_plot rqt_plot \
                           /cmd_vel/linear/x""" # C-m

# rviz
# create windows and panes
tmux new-window -t ${SESSION} -n rviz
setup_env ${SESSION}:rviz.0
# run commands
tmux send-keys -t ${SESSION}:rviz.0 """rosrun rviz rviz \
  -d \$(rospack find upnabot_path_tracking)/rviz/pursuit.rviz""" C-m


# Attach to the tmux session
tmux attach -t $SESSION:path_tracking.1

