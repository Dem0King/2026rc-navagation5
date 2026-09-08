#!/usr/bin/env python3
"""
test_points.py — 纯跑点测试 (不触发机械臂)

完全复用 nav_goal_sender.py 的导航逻辑:
  前排箱: 过渡点1 → 物资箱 → [停4s] → 过渡点2/3 → 归位区 → [停4s]
  后排箱: 物资箱 → [停4s] → 过渡点2/3 → 归位区 → [停4s]

用法:
  rosrun ladio_nav test_points.py
  rosrun ladio_nav test_points.py _start_index:=0 _max_boxes:=4
"""

import rospy
import sys
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# ============================================================
# 过渡点 (与 nav_goal_sender.py 同步)
# ============================================================
# 过渡点1: 出发区前, 去前排箱之前先经过
TRANSIT1_X = 0.970
TRANSIT1_Y = -0.101
TRANSIT1_YAW = -0.060

# 过渡点2: 左边 (左两列 col 0,1 的箱去归位区前经过)
TRANSIT2_X = 4.515
TRANSIT2_Y = 1.250
TRANSIT2_YAW = 0.031

# 过渡点3: 右边 (右两列 col 2,3 的箱去归位区前经过)
TRANSIT3_X = 4.610
TRANSIT3_Y = -0.804
TRANSIT3_YAW = -0.302

# ============================================================
# 物资箱点位 (与 nav_goal_sender.py 同步)
# 格式: (x, y, yaw, box_type, is_front, column)
# ============================================================
BOXES = [
    # ---- 前排 前排 前排 (1箱) 前排 前排 前排----
    ( 1.329, 1.060, 0.040, 0, True,  0),  # 前排第0列
    # ( 1.829, 1.260, 0.040, 1, True,  0),  # 前排第0列
    # ( 1.829, 1.260, 0.040, 2, True,  0),  # 前排第0列
    # ( 1.829, 1.260, 0.040, 3, True,  0),  # 前排第0列

    # ( 1.769, 0.458, 0.023, 0, True,  1),  # 前排第1列
    # ( 1.769, 0.458, 0.023, 1, True,  1),  # 前排第1列
    # ( 1.769, 0.458, 0.023, 2, True,  1),  # 前排第1列
    # (1.769, 0.458, 0.023, 3, True,  1),  # 前排第1列

    # ( 1.809, -0.328, -0.003, True,  2),  # 前排第2列
    # ( 1.809, -0.328, -0.003, 1, True,  2),  # 前排第2列
    # ( 1.809, -0.328, -0.003, 2, True,  2),  # 前排第2列
    # ( 1.809, -0.328, -0.003, 3, True,  2),  # 前排第2列

    # ( 1.706, -1.210, -0.016, 0, True,  3),  # 前排第3列
    # ( 1.706, -1.210, -0.016, 1, True,  3),  # 前排第3列
    # ( 1.706, -1.210, -0.016, 2, True,  3),  # 前排第3列
    # ( 1.706, -1.210, -0.016, 3, True,  3),  # 前排第3列

#           box_type: 0=药品 1=工具 2=食品 3=仪器

    # ---- 后排 后排 后排 (3箱, 除被撞开那列) 后排 后排 后排----
    # ( 3.689, 1.323,    3.075,     0, False, 0),  # 后排第0列
    # ( 3.689, 1.323,    3.075,     1, False, 0),  # 后排第0列
    # ( 3.689, 1.323,    3.075,     2, False, 0),  # 后排第0列
    # ( 3.689, 1.323,    3.075,     3, False, 0),  # 后排第0列

    ( 3.912, 0.441,    -3.131,     0, False, 1),  # 后排第1列
    # ( 3.712, 0.441,    -3.131,     1, False, 1),  # 后排第1列
    # ( 3.712, 0.441,    -3.131,     2, False, 1),  # 后排第1列
    # ( 3.712, 0.441,    -3.131,     3, False, 1),  # 后排第1列

    ( 3.971, -0.489,    -3.070,     1, False, 2),  # 后排第2列
    # ( 3.771, -0.489,    -3.070,     1, False, 2),  # 后排第2列
    # ( 3.771, -0.489,    -3.070,     2, False, 2),  # 后排第2列
    # ( 3.771, -0.489,    -3.070,     3, False, 2),  # 后排第2列

    ( 3.958, -1.288,    3.125,     3, False, 3),  # 后排第3列
    # ( 3.738, -1.288,    3.125,     1, False, 3),  # 后排第3列
    # ( 3.738, -1.288,    3.125,     2, False, 3),  # 后排第3列
    # ( 3.738, -1.288,    3.125,     3, False, 3),  # 后排第3列
]

# ============================================================
# 归位区点位 (与 nav_goal_sender.py 同步)
# ============================================================
ZONES = [
    ( 4.937, 1.243, 0.049),   # 归位区 0: 食品 (绿色)
    ( 5.000, 0.477, 0.046),   # 归位区 1: 工具 (灰色)
    ( 5.004, -0.346, 0.029),   # 归位区 2: 仪器 (蓝色)
    ( 4.996, -1.156, 0.006),   # 归位区 3: 药品 (红色)
]

YOLO_CLASS_TO_ZONE = {
    0: 3,  # 药品 → 区3
    1: 1,  # 工具 → 区1
    2: 0,  # 食品 → 区0
    3: 2,  # 仪器 → 区2
}

BOX_TYPE_NAMES = {0: "药品", 1: "工具", 2: "食品", 3: "仪器"}
ZONE_NAMES     = {0: "绿色(食品)", 1: "灰色(工具)", 2: "蓝色(仪器)", 3: "红色(药品)"}


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
        rospy.logerr("[导航] %s 超时 (%.0fs)!", label, timeout)
        client.cancel_goal()
        return False
    state = client.get_state()
    if state != actionlib.GoalStatus.SUCCEEDED:
        rospy.logerr("[导航] %s 失败! state=%d", label, state)
        return False
    rospy.loginfo("[导航] %s 到达! 停 4s...", label)
    rospy.sleep(4.0)
    return True


def main():
    rospy.init_node("test_points", anonymous=True)

    # ---- 参数 ----
    start_index = rospy.get_param("~start_index", 0)
    max_boxes   = rospy.get_param("~max_boxes", len(BOXES))
    nav_timeout = rospy.get_param("~nav_timeout", 180.0)

    boxes_to_run = BOXES[start_index : start_index + max_boxes]

    rospy.loginfo("=" * 55)
    rospy.loginfo("纯跑点测试 (不触发机械臂) — 复用比赛导航逻辑")
    rospy.loginfo("  共 %d 个点位 (从第 %d 个开始)", len(boxes_to_run), start_index + 1)
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        row_str = "前排" if is_front else "后排"
        side_str = "左→过渡点2" if col <= 1 else "右→过渡点3"
        rospy.loginfo("  %d. %s 第%d列 %s → 区%d %s [%s]",
                      start_index + i + 1, row_str, col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id], side_str)
    rospy.loginfo("=" * 55)

    # ---- 连接 move_base ----
    rospy.loginfo("[初始化] 连接 move_base...")
    nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    if not nav_client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("move_base 连接失败!")
        sys.exit(1)
    rospy.loginfo("[初始化] move_base 已连接")

    # ============================================================
    # 逐个跑 (不触发机械臂)
    # ============================================================
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        box_num = start_index + i + 1
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        zx, zy, zyaw = ZONES[zone_id]

        rospy.loginfo("=" * 55)
        rospy.loginfo("[%d/%d] %s 第%d列 %s → 归位区%d %s",
                      box_num, len(boxes_to_run),
                      "前排" if is_front else "后排", col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id])

        # ---- 前排箱: 过渡点1 → 物资箱 ----
        if is_front:
            if not navigate_to(nav_client, TRANSIT1_X, TRANSIT1_Y, TRANSIT1_YAW,
                               "过渡点1(前排)", nav_timeout):
                continue

        # ---- 去物资箱 ----
        if not navigate_to(nav_client, bx, by, byaw,
                           "物资箱%d(%s)" % (box_num, BOX_TYPE_NAMES[btype]), nav_timeout):
            continue

        # ---- 按列分左右 → 过渡点2或3 → 归位区 ----
        if col <= 1:
            if not navigate_to(nav_client, TRANSIT2_X, TRANSIT2_Y, TRANSIT2_YAW,
                               "过渡点2(左)", nav_timeout):
                continue
        else:
            if not navigate_to(nav_client, TRANSIT3_X, TRANSIT3_Y, TRANSIT3_YAW,
                               "过渡点3(右)", nav_timeout):
                continue

        # ---- 去归位区 ----
        if not navigate_to(nav_client, zx, zy, zyaw,
                           "归位区%d %s" % (zone_id, ZONE_NAMES[zone_id]), nav_timeout):
            continue

        rospy.loginfo("✅ [%d/%d] 完成!", box_num, len(boxes_to_run))

    rospy.loginfo("=" * 55)
    rospy.loginfo("跑点测试结束!")
    rospy.loginfo("=" * 55)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
