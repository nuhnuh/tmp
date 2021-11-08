#!/bin/bash


# set the frame of the path
frame_id=map # frame_id in [map, odom]
# argv[1] especifies the frame of the path (default: map)
argc=$#
if [ ! $argc -eq 0 ] ; then
  #echo "input arguments: $@"
  frame_id=$1 # frame_id in [map, odom]
fi


#
echo "checking dependencies .."
rospack find path_tracking > /dev/null
if [ ! $? -eq 0 ] ; then
  echo "path_tracking ROS package not found!! (did you forgot running source devel/setup.bash?)"
  #break
  exit 1
fi
echo ".. dependencies are OK"



echo "generating paths to track .."
python <<-EOF
		from path_tracking.geometry2d import *
		
		# define path specs
		npoints = 10
		x = np.linspace(-1.0, +1.0, npoints)
		y = np.linspace(   0,    0, npoints)
		
		# create the paths
		# path 1
		path1 = LinearPWPath(x,y)
		# path 2
		path2 = PathUtils.invert_direction(path1)
		
		# scale
		scale = 1.0
		path1 = PathUtils.scale(path1, scale, scale)
		path2 = PathUtils.scale(path2, scale, scale)
		
		# save paths
		path1.save('/tmp/path_1.npz')
		path2.save('/tmp/path_2.npz')
		
		## visualize
		#import matplotlib.pyplot as plt
		#plt.figure(figsize=(9.5,4))
		#plt.plot(path1.x,     path1.y,     'x-b', label='path1')
		#plt.plot(path1.x[-1], path1.y[-1],  'ob', label='goal1')
		#plt.plot(path2.x,     path2.y,     '+:r', label='path2')
		#plt.plot(path2.x[-1], path2.y[-1],  'or', label='goal2')
		#plt.legend()
		#plt.axis('equal')
		#plt.show()
EOF
if [ ! $? -eq 0 ] ; then
  echo "error generating the paths"
  exit 1
fi
echo ".. paths ready"


# track path sequence

#echo "loop starts .."
#while true; do

echo "############## path 1"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_1.npz _frame_id:=$frame_id _reverse:=False
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.5

echo "############## path 2"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_2.npz _frame_id:=$frame_id _reverse:=True
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
  sleep 0.5

#done
#echo ".. loop ends"


exit 0
