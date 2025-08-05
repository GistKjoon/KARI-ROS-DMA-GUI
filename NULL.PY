#!/usr/bin/env python3
# --------------------------------------------------------------------------
# UAV_MURTAP_NULL.py — three-agent test (KARI_4 · KARI_5 · KARI_6)
# --------------------------------------------------------------------------
# • x:=5 → KARI_4, 6 → KARI_5, 7 → KARI_6
# • 모든 에이전트가 서로를 이웃(neighbors)으로 사용
# • 초기 GPS 좌표는 하드코딩하지 않음(필요하면 --ros-args 로 my_coord 주입)
# --------------------------------------------------------------------------

import os
from math import cos, radians
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ──────────────────────────────────────────────────────────────────────────
# 에이전트 리스트 (순서는 x-2 = 인덱스와 매칭)
# ──────────────────────────────────────────────────────────────────────────
ALL_AGENTS = ["KARI_4", "KARI_5", "KARI_6", "KARI_7", "KARI_8"]  # x=5,6,7 → idx 4,5,6


def _launch_setup(context, *args, **kwargs):
    # 0) 런치 인자 x ↔ KARI_n 매핑 -----------------------------------------
    x_val = int(LaunchConfiguration("x").perform(context))  # 5,6,7 기대
    if x_val not in (5, 6, 7, 8, 9):
        raise RuntimeError("x must be 5 (KARI_4), 6 (KARI_5) or 7 (KARI_6)")

    agent_idx = x_val - 1                 # 5→4, 6→5, 7→6
    agent_id  = f"KARI_{agent_idx}"

    # 1) 이웃 목록 = 나를 제외한 전체 --------------------------------------
    neighbors = [aid for aid in ALL_AGENTS if aid != agent_id]

    # 2) PX4 UAV 노드 ------------------------------------------------------
    px_uav = Node(
        package="px4_uv",
        executable="pxUAV",
        name="pxUAV",
        namespace=agent_id,
        output="screen",
        parameters=[{
            "AgentID": agent_id,
            "Payload": "NULL",
        }],
    )

    # 3) MURTAP 노드 -------------------------------------------------------
    murtap_params = {
        "AgentID":    agent_id,
        "UVtype":     "UAV",
        "Payload":    "NULL",
        "neighbors":  neighbors,
        "num_agents": len(ALL_AGENTS),
        # 'my_coord' 파라미터는 필요 시 launch 호출 시 주입
    }

    murtap = Node(
        package="dynma",
        executable="MURTAP",
        name="MURTAP",
        namespace=agent_id,
        output="screen",
        parameters=[murtap_params],
    )

    return [px_uav, murtap]


def generate_launch_description():
    # 0) Gazebo 모델 경로 환경변수 -----------------------------------------
    gz_model_path = os.path.expanduser("~") + "/PX4-Autopilot/Tools/simulation/gz/models/"
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{existing}:{gz_model_path}" if existing else gz_model_path
    print("[Launch] GZ_SIM_RESOURCE_PATH:", os.environ["GZ_SIM_RESOURCE_PATH"])

    # 1) CLI 인자: x (에이전트 선택) ---------------------------------------
    x_arg = DeclareLaunchArgument(
        "x",
        default_value="5",
        description="Agent index — 5:KARI_4, 6:KARI_5, 7:KARI_6 (three-agent mode)",
    )

    # 2) XRCE-DDS Agent 실행 ----------------------------------------------
    micro_xrce_agent = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=["sudo", "MicroXRCEAgent", "serial", "--dev", "/dev/ttyUSB0", "-b", "921600"],
            output="screen",
        )],
    )

    # 3) PX4 + MURTAP 실행 -------------------------------------------------
    nodes_launch = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=_launch_setup)]
    )

    return LaunchDescription([
        x_arg,
        micro_xrce_agent,
        nodes_launch,
    ])
