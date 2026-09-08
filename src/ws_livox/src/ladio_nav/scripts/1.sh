#!/bin/bash
# ============================================================
# 2026RC 一键启动脚本
# 顺序: 雷达 → 定位 → 导航 → 深度相机 → 任务流程
# ============================================================

# echo "========================================"
# echo "  2026RC 机器狗比赛系统 — 一键启动"
# echo "========================================"

# ---- 1. 雷达驱动 ----
echo "[1/5] 启动 LiDAR 驱动..."
gnome-terminal --tab "LiDAR" -- bash -c "

roslaunch livox_ros_driver2 msg_MID360.launch
exec bash
"
sleep 6

# ---- 2. 定位 ----
echo "[2/5] 启动定位系统..."
gnome-terminal --tab "Localization" -- bash -c "

roslaunch fast_lio_localization localizetion.launch rviz:=false
exec bash
"
sleep 18

# ---- 3. 初始位姿 ----
echo "[3/5] 发布初始位姿..."
gnome-terminal --tab "InitPose" -- bash -c "

rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
exec bash
"
sleep 1

# ---- 4. 导航 ----
echo "[5/6] 启动导航 (move_base + serial_bridge)..."
gnome-terminal --tab "Navigation" -- bash -c "

roslaunch ladio_nav move_base.launch
exec bash
"
sleep 2

echo "========================================"
echo "  定位启动完成！"
echo "========================================"
