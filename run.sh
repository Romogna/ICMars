echo "You are one jive turkey!"
echo "You can't prove anything!"

export HOSTNAME="$HOSTNAME"
export T1="Marvel"
export ARDUINO_PORT="$(./get_arduino_port.sh)"

source devel/setup.bash

echo "Arduino on port $ARDUINO_PORT"
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

