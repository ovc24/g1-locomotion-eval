#!/bin/bash
cd /root/g1_ws/unitree_mujoco/simulate/build
exec env -i HOME=$HOME DISPLAY=$DISPLAY PATH=/usr/bin:/bin \
  LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./unitree_mujoco
