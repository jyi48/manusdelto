#!/bin/bash
# Manus + DG5F-M bench rig (glove + M driver + GUI).
# Usage:
#   ./launch_m.sh        — both hands
#   ./launch_m.sh left   — left hand only
#   ./launch_m.sh right  — right hand only
#
# The M keeps both hands in one namespace, separated by lj_/rj_ prefixes:
#   /dg5f_both/lj_dg_pospid/reference   (169.254.186.73)
#   /dg5f_both/rj_dg_pospid/reference   (169.254.186.72)
#
# Single-hand mode uses delto_ip, which defaults to the LEFT hand -- pass it
# explicitly for the right one (see below).

case "${1}" in
    left)
        ros2 launch manusdelto_bringup manusdelto.launch.py \
            hand_model:=m hand_ns:=dg5f_left \
            delto_ip:=169.254.186.73 "${@:2}"
        ;;
    right)
        ros2 launch manusdelto_bringup manusdelto.launch.py \
            hand_model:=m hand_ns:=dg5f_right \
            delto_ip:=169.254.186.72 "${@:2}"
        ;;
    *)
        ros2 launch manusdelto_bringup manusdelto.launch.py \
            hand_model:=m hand_ns:=dg5f_both "${@:2}"
        ;;
esac
