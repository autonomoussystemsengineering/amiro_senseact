#!/bin/bash

[[ $_ != $0 ]] && SOURCED=1 || SOURCED=0;
THIS_PWD=${PWD}
if [[ $SOURCED == 0 ]]; then
  echo "Run 'source $0'"
else
  export MUROX_ENV=${PWD}
  export MUROX_RSB_VERSION_ARM=0.13
  export MUROX_RSB_VERSION=0.17
  export MUROX_RSC_VERSION=0.17
  export MUROX_RST_VERSION=0.17
  export MUROX_DOKU=${MUROX_ENV}/dokuwiki
  export MUROX_PROJECT=${MUROX_ENV}/project
  export MUROX_ROS=${MUROX_ENV}/catkin_ws
  export MUROX_INCLUDE_DIRS=${MUROX_ENV}/project/includes
  export MUROX_CMAKE_MODULES=${MUROX_ENV}/project/utilities/cmake
  export MUROX_TYPES=${MUROX_ENV}/project/includes/types/
  export MUROX_CROSS_ENV=/opt/poky/1.7.2/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi
  export MUROX_CXX_FLAGS='-std=c++11'
  # Check if we have to build the types
  if [[ $(find ${MUROX_TYPES} -name "*pb.cc" | wc -l) -ne $(find ${MUROX_TYPES} -name "*proto" | wc -l) ]]; then
    cd ${MUROX_TYPES} && ./createPbFiles.sh ${MUROX_RST_VERSION}
    cd -
  fi
  # Source ROS
  test -e ${MUROX_ROS}/devel/ && source ${MUROX_ROS}/devel/setup.bash
fi
