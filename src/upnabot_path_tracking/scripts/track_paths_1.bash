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
		npoints = 20

		# path 5
		#
		x = np.linspace(-1.0, -0.5, npoints)
		y = np.linspace(-0.5, -0.5, npoints)
		path5_1 = LinearPWPath(x,y)
		#
		x0, y0, a0 = -0.5, -0.5, np.deg2rad(0)
		x1, y1, a1 =    0,    0, np.deg2rad(-90)
		path5_2 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, npoints)
		#
		x = np.linspace(0,   0, npoints )
		y = np.linspace(0, 0.5, npoints )
		path5_3 = LinearPWPath(x,y)
		#
		x = np.hstack(( path5_1.x[:], path5_2.x[1:], path5_3.x[1:] ))
		y = np.hstack(( path5_1.y[:], path5_2.y[1:], path5_3.y[1:] ))
		path5 = LinearPWPath(x,y)

		# path 6
		#
		x = np.linspace(  0, 0, npoints )
		y = np.linspace(0.5, 0, npoints )
		path6_1 = LinearPWPath(x,y)
		#
		x0, y0, a0 =    0,    0, np.deg2rad(+90)
		x1, y1, a1 = +0.5, -0.5, np.deg2rad(0)
		path6_2 = PathUtils.create_1segment_cspline(x0, y0, a0, x1, y1, a1, npoints)
		#
		x = np.linspace(+0.5, +1.0, npoints)
		y = np.linspace(-0.5, -0.5, npoints)
		path6_3 = LinearPWPath(x,y)
		#
		x = np.hstack(( path6_1.x[:], path6_2.x[1:], path6_3.x[1:] ))
		y = np.hstack(( path6_1.y[:], path6_2.y[1:], path6_3.y[1:] ))
		path6 = LinearPWPath(x,y)

		# path 7
		path7 = PathUtils.invert_direction(path6)

		# path 8
		path8 = PathUtils.invert_direction(path5)
		
		# scale
		scale = 1.0
		path5 = PathUtils.scale(path5, scale, scale)
		path6 = PathUtils.scale(path6, scale, scale)
		path7 = PathUtils.scale(path7, scale, scale)
		path8 = PathUtils.scale(path8, scale, scale)
		
		# save paths
		path5.save('/tmp/path_5.npz')
		path6.save('/tmp/path_6.npz')
		path7.save('/tmp/path_7.npz')
		path8.save('/tmp/path_8.npz')
		
		## visualize
		#import matplotlib.pyplot as plt
		#plt.figure(figsize=(9.5,4))
		#plt.plot(path5.x,     path5.y,     'x-r', label='path5')
		#plt.plot(path5.x[-1], path5.y[-1],  'or', label='goal5')
		#plt.plot(path6.x,     path6.y,     '+:g', label='path6')
		#plt.plot(path6.x[-1], path6.y[-1],  'og', label='goal6')
		#plt.plot(path7.x,     path7.y,     'x-b', label='path7')
		#plt.plot(path7.x[-1], path7.y[-1],  'ob', label='goal7')
		#plt.plot(path8.x,     path8.y,     '+:y', label='path8')
		#plt.plot(path8.x[-1], path8.y[-1],  'oy', label='goal8')
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

echo "############## paths 5, 6, 7 & 8"

echo "############## paths 5-6"
echo "############## path 5"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_5.npz _frame_id:=$frame_id _reverse:=False
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.0

echo "############## path 6"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_6.npz _frame_id:=$frame_id _reverse:=True
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.5

echo "############## paths 7-8"
echo "############## path 7"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_7.npz _frame_id:=$frame_id _reverse:=False
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.0

echo "############## path 8"
rosrun path_tracking path_tracking_client.py _path_fn:=/tmp/path_8.npz _frame_id:=$frame_id _reverse:=True
if [ ! $? -eq 0 ] ; then
  echo "Aborting!!"
  exit 1
fi
sleep 0.5

#done
#echo ".. loop ends"


exit 0
