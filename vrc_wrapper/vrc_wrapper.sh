#!/bin/bash

test -r /opt/ros/fuerte/setup.sh && . /opt/ros/fuerte/setup.sh
. /etc/environment
$*
