#!/bin/bash
# Manus + DG5F-M bench rig (glove + M driver + GUI).
# Usage:
#   ./launch_m.sh        — both hands
#   ./launch_m.sh left   — left hand only
#   ./launch_m.sh right  — right hand only
#
# One driver per hand, each in its own namespace:
#   /dg5f_left/lj_dg_pospid/reference    (169.254.186.73)
#   /dg5f_right/rj_dg_pospid/reference   (169.254.186.72)

case "${1}" in
    left|right) HANDS="${1}" ;;
    *)          HANDS=both ;;
esac

ros2 launch manusdelto_bringup manusdelto.launch.py \
    hand_model:=m \
    hands:="${HANDS}" \
    "${@:2}"
