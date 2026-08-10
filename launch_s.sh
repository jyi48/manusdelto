#!/bin/bash
# Manus + DG5F-S bench rig (glove + S driver + GUI).
# Usage:
#   ./launch_s.sh        — both hands
#   ./launch_s.sh left   — left hand only
#   ./launch_s.sh right  — right hand only
#
# The S has no both-hand launch (left and right share controller and joint
# names), so "both" brings up its two single-hand stacks, one namespace each:
#   /dg5f_s_left/joint_pospid/reference   (169.254.186.73)
#   /dg5f_s_right/joint_pospid/reference  (169.254.186.72)

case "${1}" in
    left)  NS=dg5f_left  ;;
    right) NS=dg5f_right ;;
    *)     NS=dg5f_both  ;;
esac

ros2 launch manusdelto_bringup manusdelto.launch.py \
    hand_model:=s \
    hand_ns:="${NS}" \
    "${@:2}"
