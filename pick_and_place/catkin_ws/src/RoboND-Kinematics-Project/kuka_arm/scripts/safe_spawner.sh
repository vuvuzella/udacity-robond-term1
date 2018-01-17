#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch kuka_arm target_description.launch &
sleep 4 &&
x-terminal-emulator -e roslaunch kuka_arm cafe.launch &
sleep 4 &&
x-terminal-emulator -e roslaunch kuka_arm spawn_target.launch &
sleep 6 &&
x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch &
# roslaunch kuka_arm inverse_kinematics.launch &
sleep 6 &&
x-terminal-emulator -e rosrun kuka_arm IK_server.py
