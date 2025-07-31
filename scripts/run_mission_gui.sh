#!/usr/bin/env bash
#────────── ROS 환경 ──────────
source /opt/ros/humble/setup.bash
source "$HOME/KARI-ROS2-DMA/install/setup.bash"

# (선택) 도메인 지정 : 필요하면 숫자만 바꿔 쓰세요
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

#────────── GUI 실행 ──────────
exec ros2 run dynma_gui mission_gui "$@"
