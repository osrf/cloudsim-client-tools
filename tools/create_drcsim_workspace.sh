#!/bin/bash

# Check out and build everything that's needed to
# get a drcsim workspace established.  Intended
# for use on cloud machines.

set -e

USAGE="create_drcsim_workspace.sh <srcdir> [<install_prefix>]"

if [ $# -lt 1 ]; then
  echo $USAGE
  exit 1
fi
DESTDIR=$1
if [ $# -gt 1 ]; then
  INSTALL_PREFIX=$2
else
  INSTALL_PREFIX=/usr
fi
#if [ -e $DESTDIR ]; then
#  echo "Destination directory $DESTDIR already exists.  Aborting."
#  exit 1
#fi

# TODO: update to point to version in repo
ROSINSTALL_FILE_URL=$HOME/code/drc/cloudsim-client-tools/tools/drcsim.rosinstall
ROSDISTRO=fuerte

# Check out everything
echo "Assembling workspace in $DESTDIR..."
#rosinstall $DESTDIR $ROSINSTALL_FILE_URL /opt/ros/$ROSDISTRO

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH

# Build gazebo
PROJECT=gazebo
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make
sudo make install

# Build osrf-common
PROJECT=osrf-common
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
eval export ROS_PACKAGE_PATH=$DESTDIR/$PROJECT:\$ROS_PACKAGE_PATH
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make
sudo make install

# Build sandia-hand
PROJECT=sandia-hand
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
eval export ROS_PACKAGE_PATH=$DESTDIR/$PROJECT:$INSTALL_PREFIX/share/osrf-common-*/ros:\$ROS_PACKAGE_PATH
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make
sudo make install

# Build drcsim
PROJECT=drcsim
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make
sudo make install
