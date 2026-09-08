#!/bin/bash
# ============================================================
# 2026RC 机器狗比赛系统 — 一键启动脚本（稳跑版）
# 顺序: ROS Master → 雷达 → 定位 → 导航 → 深度相机 → 姿态转发 → 任务流程
# ============================================================

echo "========================================"
echo "  2026RC 机器狗比赛系统 — 一键启动"
echo "========================================"

# echo "[1] 启动导航 (move_base + serial_bridge)..."
# gnome-terminal --tab "Navigation" -- bash -c "

# rosrun ladio_nav serial_bridge.py
# exec bash
# "
# sleep 2

# # ---- 4. 深度相机 ----
# echo "[2] 启动深度相机 (YOLO + 串口)..."
# gnome-terminal --tab "DepthCamera" -- bash -c "
# cd ~/RC_viewer
# source devel/setup.bash
# ./reset_gemini.sh && roslaunch yolo_depth_pkg gemini2_yolo_distance_D2C.launch
# exec bash
# "
# sleep 8



# ---- 1. 雷达驱动 ----
echo "[3] 启动 LiDAR 驱动..."
gnome-terminal --tab "LiDAR" -- bash -c "

roslaunch livox_ros_driver2 msg_MID360.launch publish_freq:=20.0
exec bash
"
sleep 7
# roslaunch livox_ros_driver2 msg_MID360.launch publish_freq:=20.0



# ---- 2. 定位 ----
echo "[4] 启动定位系统..."
gnome-terminal --tab "Localization" -- bash -c "

roslaunch fast_lio mapping_mid360.launch rviz:=true
exec bash
"
sleep 8

# # ---- 3. 初始位姿 ----
# echo "[5] 发布初始位姿..."
# gnome-terminal --tab "InitPose" -- bash -c "

# rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
# exec bash
# "
# sleep 1



# ---- 4. 导航 ----
# echo "[6] 启动导航 (move_base + serial_bridge)..."
# gnome-terminal --tab "Navigation" -- bash -c "

# roslaunch ladio_nav move_base.launch
# exec bash
# "
# sleep 2


# # ---- vlm高分题 ----
# echo "[1/5] 启动 vlm..."
# gnome-terminal --tab "vlm" -- bash -c "

# roslaunch abot_vision vlm_node.launch
# exec bash
# "
# sleep 1

# ---- vlm高分题 ----
# echo "[7] 启动 vlm..."
# gnome-terminal --tab "vlm" -- bash -c "

# echo '========================================'
# echo '  vlm高分题 vlm高分题开始识别'
# echo '========================================'
# exec bash
# "
# sleep 1

# ---- 5. 任务流程 ----
echo "[6/6] 启动完整比赛流程..."
gnome-terminal --tab "TaskRunner" -- bash -c "

echo '========================================'
echo '  所有节点已就绪，按 Enter 开始执行任务'
echo '========================================'
read
cd ~/livox_ws2/livox_ws/src/ws_livox
source devel/setup.bash
rosrun send_pose_cpp send_pose_node
exec bash"

