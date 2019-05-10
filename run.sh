echo "You are one jive turkey!"
echo "You can't prove anything!"

export HOSTNAME="$HOSTNAME"
export T1="Marvel"

source devel/setup.bash

echo "$T1 launching on $HOSTNAME"

if [ "Marvel" = "$T1" ]; then
  roslaunch icmars_onboard rr.launch
fi

if [ "Vive" = "$T1" ]; then
  echo "Vive not yet ready"
fi

if [ "Vicon" = "$T1" ]; then
  echo "Vicon is in progress"
fi

