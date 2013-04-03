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
DESTDIR=`readlink -f $1`
if [ $# -gt 1 ]; then
  INSTALL_PREFIX=`readlink -f $2`
else
  INSTALL_PREFIX=/usr
fi
if [ -e $DESTDIR ]; then
  echo "Destination directory $DESTDIR already exists.  Aborting."
  exit 1
fi

set -x

ROSINSTALL_FILE_URL=https://bitbucket.org/osrf/cloudsim-client-tools/raw/default/tools/drcsim.rosinstall
ROSDISTRO=fuerte
MAKE_J=-j16

# add ROS repo
sudo apt-get install -y wget
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu fuerte main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
# OSRF repository to get bullet
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu fuerte main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -
sudo apt-get update

# Install pre-reqs, which were gathered from the debian/control files in the
# various -release repositories
# avr-gcc avr-libc 
sudo apt-get install -y cmake freeglut3-dev libavcodec-dev libavformat-dev libboost-dev libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libbullet-dev libcegui-mk2-dev libcurl4-openssl-dev libfreeimage-dev libltdl-dev libogre-dev libprotobuf-dev libprotoc-dev libqt4-dev libswscale-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev mercurial osrf-common pkg-config protobuf-compiler robot-player-dev ros-fuerte-common-msgs ros-fuerte-console-bridge ros-fuerte-geometry ros-fuerte-geometry-experimental ros-fuerte-image-common ros-fuerte-image-pipeline ros-fuerte-pr2-controllers ros-fuerte-pr2-mechanism ros-fuerte-robot-model-visualization ros-fuerte-ros ros-fuerte-ros-comm ros-fuerte-std-msgs ros-fuerte-urdfdom ros-fuerte-xacro

# Check out everything
sudo apt-get install -y python-pip
sudo pip install rosinstall
echo "Assembling workspace in $DESTDIR..."
rosinstall $DESTDIR $ROSINSTALL_FILE_URL /opt/ros/$ROSDISTRO

export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$INSTALL_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH

# Build gazebo
PROJECT=gazebo
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make $MAKE_J
sudo make install

# Build osrf-common
PROJECT=osrf-common
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
eval export ROS_PACKAGE_PATH=$DESTDIR/$PROJECT:\$ROS_PACKAGE_PATH
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make $MAKE_J
sudo make install

# Build sandia-hand
PROJECT=sandia-hand
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
# TODO: get the osrf-common version from somewhere
eval export ROS_PACKAGE_PATH=$DESTDIR/$PROJECT:$INSTALL_PREFIX/share/osrf-common-1.0/ros:\$ROS_PACKAGE_PATH
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make $MAKE_J
sudo make install

# Build drcsim
PROJECT=drcsim
mkdir -p $DESTDIR/build/$PROJECT
cd $DESTDIR/build/$PROJECT
. /opt/ros/$ROSDISTRO/setup.bash
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX $DESTDIR/$PROJECT
make $MAKE_J
sudo make install

# Generate a convenience file if we installed to somewhere other than /usr
if [ $INSTALL_PREFIX != /usr ]; then
  cat > /tmp/drcsim-setup.sh <<DELIM
#!/bin/bash
export PATH=$INSTALL_PREFIX/bin:$PATH
export LD_LIBRARY_PATH=$INSTALL_PREFIX/lib:$LD_LIBRARY_PATH
. $INSTALL_PREFIX/share/drcsim/setup.sh
DELIM
  sudo mv /tmp/drcsim-setup.sh $INSTALL_PREFIX/drcsim-setup.sh
  echo "To configure your shell:"
  echo "  . $INSTALL_PREFIX/drcsim-setup.sh"
else
  echo "To configure your shell:"
  echo "  . /usr/share/drcsim/setup.sh"
fi
