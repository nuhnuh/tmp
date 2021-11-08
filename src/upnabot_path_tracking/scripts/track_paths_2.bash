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
		
		# create paths
		
		# path 3
		#
		x0, y0, a0 = -1.3, 0, np.deg2rad(0)
		x1, y1, a1 = -0.9, 0, np.deg2rad(0)
		path3_1 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 5)
		#
		x0, y0, a0 = -0.9,     0, np.deg2rad(0)
		x1, y1, a1 = -0.4, -0.25, np.deg2rad(-45)
		path3_2 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = -0.4, -0.25, np.deg2rad(-45)
		x1, y1, a1 = +0.0, -0.50, np.deg2rad(0)
		path3_3 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = +0.0, -0.50, np.deg2rad(0)
		x1, y1, a1 = +0.4, -0.25, np.deg2rad(+90)
		path3_4 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = +0.4, -0.25, np.deg2rad(+90)
		x1, y1, a1 = -0.4, +0.25, np.deg2rad(+90)
		path3_5 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 60)
		#
		x0, y0, a0 = -0.4, +0.25, np.deg2rad(+90)
		x1, y1, a1 = -0.0, +0.50, np.deg2rad(0)
		path3_6 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = -0.0, +0.50, np.deg2rad(0)
		x1, y1, a1 = +0.4, +0.25, np.deg2rad(-45)
		path3_7 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = +0.4, +0.25, np.deg2rad(-45)
		x1, y1, a1 = +0.9,     0, np.deg2rad(0)
		path3_8 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 20)
		#
		x0, y0, a0 = +0.9, 0, np.deg2rad(0)
		x1, y1, a1 = +1.3, 0, np.deg2rad(0)
		path3_9 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, 5)
		#
		x = np.hstack(( path3_1.x[:], path3_2.x[1:], path3_3.x[1:], path3_4.x[1:], path3_5.x[1:], path3_6.x[1:], path3_7.x[1:], path3_8.x[1:], path3_9.x[1:]))
		y = np.hstack(( path3_1.y[:], path3_2.y[1:], path3_3.y[1:], path3_4.y[1:], path3_5.y[1:], path3_6.y[1:], path3_7.y[1:], path3_8.y[1:], path3_9.y[1:]))
		path3 = LinearPWPath(x,y)
		
		# scale
		scale = 1.0
		path3 = PathUtils.scale(path3, scale, scale)
		
		# path 4
		path4 = PathUtils.invert_direction(path3)
		
		# save paths
		path3.save('/tmp/path_3.npz')
		path4.save('/tmp/path_4.npz')
		
		## visualize
		#import matplotlib.pyplot as plt
		#plt.figure(figsize=(9.5,4))
		#plt.plot(path3.x,     path3.y,     'x-r', label='path3')
		#plt.plot(path3.x[-1], path3.y[-1],  'or', label='goal3')
		#plt.plot(path4.x,     path4.y,     '+:g', label='path4')
		#plt.plot(path4.x[-1], path4.y[-1],  'og', label='goal4')
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

echo "############## paths 3 & 4"

echo "############## path 3"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_3.npz _frame_id:=$frame_id _reverse:=False
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.5

echo "############## path 4"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_4.npz _frame_id:=$frame_id _reverse:=True
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.5

#done
#echo ".. loop ends"


exit 0
