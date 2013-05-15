#!/bin/bash

# Set the private environment for testing the VRC contest

USAGE="Usage: set_vrc_private.sh <bitbucket_key>"

if [ $# -ne 1 ]; then
  echo $USAGE
  exit 1
fi

# Constants
GAZEBO_MODELS_NAME=gazebo_models
GAZEBO_INSTALL_DIR=/home/$USER/.gazebo
VRC_ARENAS_NAME=vrc_arenas
VRC_ARENA_INSTALL_DIR=/home/$USER/local
DRCSIM_SETUP=/usr/local/share/drcsim/setup.sh

# arg1: Name of the repository to install
# arg2: Destination directory
# arg3: ssh bitbucket key
install ()
{
    # Temporal directory for the repository
    TMP_DIR=`mktemp -d`
    cd $TMP_DIR
   
    echo -n "Downloading $1..."
    hg clone -e "ssh -o StrictHostKeyChecking=no -i $3" ssh://hg@bitbucket.org/osrf/$1
    echo "Done"
    cd $1
    mkdir build
    cd build
    echo -n "Installing $1..."
    cmake .. -DCMAKE_INSTALL_PREFIX=$2
    make install > /dev/null 2>&1
    echo "Done"

    # Remove temp dir
    rm -rf $TMP_DIR
}


KEY=$1

# gazebo_models
install $GAZEBO_MODELS_NAME $GAZEBO_INSTALL_DIR $KEY

# vrc_arenas
install $VRC_ARENAS_NAME $VRC_ARENA_INSTALL_DIR $KEY

# Add private vrc_arenas setup.sh to the drcsim setup.sh if possible
#echo -e "\nReady. Do not forget to source the new VRC Arena:"
#echo -e "\t. $VRC_ARENA_INSTALL_DIR/share/vrc_arenas/setup.sh"
