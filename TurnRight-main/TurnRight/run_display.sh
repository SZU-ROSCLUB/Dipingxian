#!/bin/bash
export DISPLAY=:0
export XAUTHORITY=/home/sunrise/.Xauthority
# 确保 root 有授权（防止重启后令牌失效）
su sunrise -c "DISPLAY=:0 xhost +local:root" 2>/dev/null
cd /userdata/TurnRight
python voice.py
