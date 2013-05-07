#!/bin/bash

# Stop a netwatcher session

USAGE="Usage: stop_netwatcher.sh [<ROS_stop_netwatcher_topic>]"
DEFAULT_STOP="/vrc/state/stop"

if [ $# -eq 0 ]; then
  STOP_TOPIC=$DEFAULT_STOP
elif [ $# -eq 1 ]; then
  STOP_TOPIC=$1
else
  echo $USAGE
  exit 1
fi

if [ ! -f /opt/ros/fuerte/setup.sh ];
then
   echo "ROS cannot be initialized (/opt/ros/fuerte/setup.sh not found)"
   exit 1
fi

echo $STOP_TOPIC

. /opt/ros/fuerte/setup.sh
rostopic pub -1 $STOP_TOPIC std_msgs/Empty > /dev/null
echo "[stop_netwatcher]: Stop message sent"
