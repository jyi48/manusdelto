#!/bin/bash
# Manus + DG5F-S bench rig (glove + S driver + GUI).
# Usage:
#   ./launch_s.sh        — both hands
#   ./launch_s.sh left   — left hand only
#   ./launch_s.sh right  — right hand only
#
# One driver per hand, each in its own namespace:
#   /dg5f_s_left/joint_pospid/reference   (169.254.186.73)
#   /dg5f_s_right/joint_pospid/reference  (169.254.186.72)

case "${1}" in
    left|right) HANDS="${1}" ;;
    *)          HANDS=both ;;
esac

ros2 launch manusdelto_bringup manusdelto.launch.py \
    hand_model:=s \
    hands:="${HANDS}" \
    "${@:2}"
