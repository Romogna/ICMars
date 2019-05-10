source ~/icmars_onboard/devel/setup.bash
#export ROS_IP=`hostname -I`
roslaunch mars_camera mars_camera.launch name:="$HOSTNAME"
