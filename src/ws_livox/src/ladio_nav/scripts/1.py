#!/usr/bin/env python3
"""
1.py — 纯跑点测试 (不触发机械臂, 不复用 arm/align/detect 代码)

完全复用 nav_goal_sender.py 的导航路径:
  前排箱: 过渡点1 → 物资箱 [停2s] → 过渡点2/3 → 归位区 [停2s]
  后排箱: 物资箱 [停2s] → 过渡点2/3 → 归位区 [停2s]

用法:
  rosrun ladio_nav 1.py
  rosrun ladio_nav 1.py _start_index:=0 _max_boxes:=4
"""

import rospy
import sys
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# ============================================================
# 以下数据与 nav_goal_sender.py 保持同步
# ============================================================
BOX_TYPE_NAMES = {0: "药品", 1: "工具", 2: "食品", 3: "仪器"}
ZONE_NAMES     = {0: "绿色(食品)", 1: "灰色(工具)", 2: "蓝色(仪器)", 3: "红色(药品)"}

YOLO_CLASS_TO_ZONE = {
    0: 3, 1: 1, 2: 0, 3: 2,
}

# 过渡点
TRANSIT1_X = 0.970;  TRANSIT1_Y = -0.101;  TRANSIT1_YAW = -0.060  # 前排箱前
TRANSIT2_X = 4.775;  TRANSIT2_Y =  1.250;  TRANSIT2_YAW =  0.031  # 左侧
TRANSIT3_X = 4.775;  TRANSIT3_Y = -0.952;  TRANSIT3_YAW = -0.004  # 右侧

# 物资箱: (x, y, yaw, box_type, is_front, column)
BOXES = [
    # ---- 前排 前排 前排 (1箱) 前排 前排 前排----
    ( 1.533, 1.043, 0.040, 0, True,  0),  # 前排第1列
    # ( 1.533, 1.260, 0.040, 1, True,  0),  # 前排第1列
    # ( 1.533, 1.260, 0.040, 2, True,  0),  # 前排第1列
    # ( 1.533, 1.260, 0.040, 3, True,  0),  # 前排第1列

    ( 1.533, 0.343, 0.023, 0, True,  1),  # 前排第2列
    # ( 1.533, 0.458, 0.023, 1, True,  1),  # 前排第2列
    # ( 1.533, 0.458, 0.023, 2, True,  1),  # 前排第2列
    # ( 1.533, 0.458, 0.023, 3, True,  1),  # 前排第2列

    # ( 1.533, -0.457, 0.054, 0, True,  2),  # 前排第3列
    # ( 1.533, -0.328, -0.003, 1, True,  2),  # 前排第3列
    # ( 1.533, -0.457, 0.054, 2, True,  2),  # 前排第3列
    # ( 1.533, -0.328, -0.003, 3, True,  2),  # 前排第3列

    # ( 1.533, -1.157, -0.016, 0, True,  3),  # 前排第4列
    # ( 1.533, -1.210, -0.016, 1, True,  3),  # 前排第4列
    # ( 1.533, -1.210, -0.016, 2, True,  3),  # 前排第4列
    # ( 1.533, -1.210, -0.016, 3, True,  3),  # 前排第4列

#           box_type: 0=药品 1=工具 2=食品 3=仪器

    # ---- 后排 后排 后排 (3箱, 除被撞开那列) 后排 后排 后排----
    # ( 3.962, 1.323,    3.075,     0, False, 0),  # 后排第1列
    # ( 3.689, 1.323,    3.075,     1, False, 0),  # 后排第1列
    # ( 3.689, 1.323,    3.075,     2, False, 0),  # 后排第1列
    # ( 3.689, 1.323,    3.075,     3, False, 0),  # 后排第1列

    # ( 3.912, 0.441,    -3.131,     0, False, 1),  # 后排第2列
    # ( 3.712, 0.441,    -3.131,     1, False, 1),  # 后排第2列
    # ( 3.712, 0.441,    -3.131,     2, False, 1),  # 后排第2列
    # ( 3.962, 0.441,    -3.131,     3, False, 1),  # 后排第2列

    # ( 3.971, -0.489,    -3.070,     0, False, 2),  # 后排第3列
    ( 2.818, -0.621,    -3.070,     1, False, 2),  # 后排第3列
    # ( 3.771, -0.489,    -3.070,     2, False, 2),  # 后排第3列
    # ( 3.771, -0.489,    -3.070,     3, False, 2),  # 后排第3列

    # ( 3.958, -1.288,    3.125,     0, False, 3),  # 后排第4列
    # ( 3.738, -1.288,    3.125,     1, False, 3),  # 后排第4列
    # ( 3.738, -1.288,    3.125,     2, False, 3),  # 后排第4列
    # ( 3.738, -1.288,    3.125,     3, False, 3),  # 后排第4列
]

# 出发点 (建图原点)
START_X = 0.0;  START_Y = 0.0;  START_YAW = 0.0

# 归位区: (x, y, yaw)
ZONES = [
    ( 5.229,  1.263,  0.013),  # 区0: 食品 (绿色)
    ( 5.229,  0.443,  0.010),  # 区1: 工具 (灰色)
    ( 5.229, -0.352, -0.004),  # 区2: 仪器 (蓝色)
    ( 5.229, -1.157, -0.001),  # 区3: 药品 (红色)
]

PAUSE_SEC = 2.0  # 每到一个点停几秒


def build_goal(x, y, yaw):
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "map"
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, yaw)
    g.target_pose.pose.orientation.x = q[0]
    g.target_pose.pose.orientation.y = q[1]
    g.target_pose.pose.orientation.z = q[2]
    g.target_pose.pose.orientation.w = q[3]
    return g


def navigate_to(client, x, y, yaw, label, timeout):
    rospy.loginfo("[导航] 前往 %s: x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)",
                  label, x, y, yaw, yaw * 57.2958)
    goal = build_goal(x, y, yaw)
    client.send_goal(goal)
    finished = client.wait_for_result(timeout=rospy.Duration(timeout))
    if not finished:
        rospy.logerr("[导航] %s 超时!", label)
        client.cancel_goal()
        return False
    state = client.get_state()
    if state != actionlib.GoalStatus.SUCCEEDED:
        rospy.logerr("[导航] %s 失败! state=%d", label, state)
        return False
    rospy.loginfo("[导航] %s 到达! 停 %.0fs...", label, PAUSE_SEC)
    rospy.sleep(PAUSE_SEC)
    return True


def main():
    rospy.init_node("test_nav_only", anonymous=True)

    start_index = rospy.get_param("~start_index", 0)
    max_boxes   = rospy.get_param("~max_boxes", len(BOXES))
    nav_timeout = rospy.get_param("~nav_timeout", 180.0)

    boxes_to_run = BOXES[start_index : start_index + max_boxes]

    rospy.loginfo("=" * 55)
    rospy.loginfo("纯跑点测试 (不触发机械臂) — 复刻比赛导航路径")
    rospy.loginfo("  共 %d 个点位 (从第 %d 个开始)", len(boxes_to_run), start_index + 1)
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        row_str = "前排" if is_front else "后排"
        side_str = "左→TP2" if col <= 1 else "右→TP3"
        rospy.loginfo("  %d. %s c%d %s → 区%d %s [%s]",
                      start_index + i + 1, row_str, col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id], side_str)
    rospy.loginfo("=" * 55)

    rospy.loginfo("[初始化] 连接 move_base...")
    nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    if not nav_client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("move_base 连接失败!")
        sys.exit(1)
    rospy.loginfo("[初始化] move_base 已连接")

    # ============================================================
    # 逐箱跑点 (无机械臂, 无视觉对准, 无YOLO识别)
    # ============================================================
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        box_num = start_index + i + 1
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        zx, zy, zyaw = ZONES[zone_id]

        rospy.loginfo("=" * 55)
        rospy.loginfo("[%d/%d] %s c%d %s → 区%d %s",
                      box_num, len(boxes_to_run),
                      "前排" if is_front else "后排", col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id])

        # 前排箱: 过渡点1 → 物资箱
        if is_front:
            if not navigate_to(nav_client, TRANSIT1_X, TRANSIT1_Y, TRANSIT1_YAW,
                               "TP1(前排)", nav_timeout):
                continue

        # 去物资箱
        if not navigate_to(nav_client, bx, by, byaw,
                           "箱%d(%s)" % (box_num, BOX_TYPE_NAMES[btype]), nav_timeout):
            continue

        # 直接去归位区 (与 nav_goal_sender 一致)
        if not navigate_to(nav_client, zx, zy, zyaw,
                           "区%d %s" % (zone_id, ZONE_NAMES[zone_id]), nav_timeout):
            continue

        rospy.loginfo("=== [%d/%d] 完成! ===", box_num, len(boxes_to_run))

    # ---- 回到出发点 ----
    rospy.loginfo("=" * 55)
    rospy.loginfo("返回出发点 (%.0f, %.0f)...", START_X, START_Y)
    navigate_to(nav_client, 0.155, 0.098, START_YAW, "出发点", nav_timeout)
    rospy.loginfo("=" * 55)
    rospy.loginfo("跑点测试结束!")
    rospy.loginfo("=" * 55)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
