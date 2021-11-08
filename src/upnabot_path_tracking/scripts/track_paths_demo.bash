#!/bin/bash


# check ROS dependencies
echo "Checking dependencies .."
rospack find path_tracking > /dev/null
if [ ! $? -eq 0 ] ; then
  echo "path_tracking ROS package not found!! (did you forget running source devel/setup.bash?)"
  #break
  exit 1
fi
echo ".. dependencies are OK"


# get the map from the robot
echo "Retrieve map from the robot .."
scp 10.66.0.1:Git/kobuki_ws/src/upnabot/upnabot_navigation/maps/demo.lidar.* $(rospack find upnabot_navigation)/maps/
if [ ! $? -eq 0 ] ; then
  echo "Error retrieving map from the robot => Aborting!!"
  exit 1
fi
# convert map to .png format
rm -f /tmp/demo.lidar.*
convert $(rospack find upnabot_navigation)/maps/demo.lidar.pgm /tmp/demo.lidar.pgm.png
if [ ! $? -eq 0 ] ; then
  echo "Error converting map to png => Aborting!!"
  exit 1
fi
ln -s $(rospack find upnabot_navigation)/maps/demo.lidar.yaml /tmp/demo.lidar.yaml
if [ ! $? -eq 0 ] ; then
  echo "Error copying yaml => Aborting!!"
  exit 1
fi


echo "Generating paths to track .."
rm -f /tmp/path_demo.npz # delete output from previous runs
echo "TIPs:"
echo "1) run the import_map script"
echo "2) edit the bezier curve to create the desired path"
echo "3) convert the curve into a mesh (F3, mesh from curve)"
echo "4) run the save_path script"
echo "4) close blender"
sleep 1
~/tmp/bin/blender-2.90.1-linux64/blender $(rospack find upnabot_path_tracking)/scripts/blender_path_editor.blend
echo "Checking that the path has been created .."
if [ ! -e /tmp/path_demo.npz ] ; then
  echo "/tmp/path_demo.npz not found!! (blender failed?)"
  exit 1
fi
echo "path generated"


echo "generating backward path"
python <<-EOF
		from path_tracking.geometry2d import *
		
		# load forward path
		path1 = LinearPWPath.load('/tmp/path_demo.npz')
		
		# invert path
		path2 = PathUtils.invert_direction(path1)
		
		# save path
		path2.save('/tmp/path_demo.backward.npz')
EOF
if [ ! $? -eq 0 ] ; then
  echo "error generating the paths"
  exit 1
fi
echo ".. backward path ready"


# path tracking loop
while true; do

  echo "############## path 1"
  rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_demo.npz _frame_id:=map _reverse:=False
  if [ ! $? -eq 0 ] ; then
    echo "Aborting!!"
    exit 1
  fi
  sleep 1.0

  echo "############## path 2"
  rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_demo.backward.npz _frame_id:=map _reverse:=True
  if [ ! $? -eq 0 ] ; then
    echo "Aborting!!"
    exit 1
  fi
  sleep 1.0

done
echo ".. loop ends"


exit 0
