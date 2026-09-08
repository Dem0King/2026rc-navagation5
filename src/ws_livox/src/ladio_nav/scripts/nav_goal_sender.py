#!/usr/bin/env python3
"""
nav_goal_sender.py — 2026RC 完整比赛流程

比赛策略 (4箱):
  - 前排1箱 + 后排3箱 (撞开前排所在列的后排箱)
  - 过渡点1: 出发区前, 去前排箱之前先经过
  - 过渡点2: 左边, 左两列(0,1)箱去归位区前先经过
  - 过渡点3: 右边, 右两列(2,3)箱去归位区前先经过

逐箱流程:
  1. [前排箱] 过渡点1 → 物资箱 → 抓取(pitch=1)
  2. [后排箱] 物资箱 → 抓取(pitch=1)
  3. 过渡点2/3 (按列分左右) → 归位区 → 放置(pitch=4/6)
  4. 循环

用法:
  rosrun ladio_nav nav_goal_sender.py
  rosrun ladio_nav nav_goal_sender.py _start_index:=0
  rosrun ladio_nav nav_goal_sender.py _max_boxes:=4
"""

import rospy
import sys
import time
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

# ============================================================
# 物资箱类型 → 归位区 映射 (YOLO class_id 一致)
# ============================================================
BOX_TYPE_NAMES = {0: "药品", 1: "工具", 2: "食品", 3: "仪器"}
ZONE_NAMES     = {0: "绿色(食品)", 1: "灰色(工具)", 2: "蓝色(仪器)", 3: "红色(药品)"}

# YOLO class_id → 比赛归位区编号
# YOLO: 0=药品 1=工具 2=食品 3=仪器
# 比赛: 0=食品 1=工具 2=仪器 3=药品
YOLO_CLASS_TO_ZONE = {
    0: 3,  # 药品 → 区3 (红色)
    1: 1,  # 工具 → 区1 (灰色)
    2: 0,  # 食品 → 区0 (绿色)
    3: 2,  # 仪器 → 区2 (蓝色)
}

# ============================================================
# 过渡点
# ============================================================
# 过渡点1: 出发区前, 去前排箱之前先经过
TRANSIT1_X = 0.970
TRANSIT1_Y = 0.069
TRANSIT1_YAW = -0.060

# 过渡点2: 左边 (左两列 col 0,1 的箱去归位区前经过)
TRANSIT2_X = 4.775
TRANSIT2_Y = 1.250
TRANSIT2_YAW = 0.031

# 过渡点3: 右边 (右两列 col 2,3 的箱去归位区前经过)
TRANSIT3_X = 4.775
TRANSIT3_Y = -0.952
TRANSIT3_YAW = -0.004

# ============================================================
# 物资箱点位 (用 rosrun tf tf_echo map body_foot 获取)
# 格式: (x_m, y_m, yaw_rad, box_type, is_front, column)
#   box_type: 0=药品 1=工具 2=食品 3=仪器
#   is_front: True=前排(需经过渡点1), False=后排
#   column:   0-3, 左边两列(0,1)→过渡点2, 右边两列(2,3)→过渡点3
# ============================================================
BOXES = [
    # ---- 前排 前排 前排 (1箱) 前排 前排 前排----
    # ( 1.525, 1.252, -0.024, 0, True,  0),  # 前排第1列 0→区0 药品(红)


    ( 1.534, 0.402, -0.034, 3, True,  1),  # 前排第2列 3→区1 仪器(蓝)


    # ( 1.523, -0.471, -0.023, 1, True,  2),  # 前排第3列 1→区2 工具(灰)


    # ( 1.495, -1.319, -0.045, 2, True,  3),  # 前排第4列 2→区3 食品(绿)

    
#           box_type: 0=药品 1=工具 2=食品 3=仪器

    # ---- 后排 后排 后排 (3箱, 除被撞开那列) 后排 后排 后排----

    # ( 3.375, -0.569,    -3.070,     2, False, 2),  # 后排第3列 2→区0 食品(绿) 3
    # ( 3.375, -0.569,    -3.070,     1, False, 2),  # 后排第3列 1→区1 工具(灰) 3
    # ( 3.375, -0.569,    -3.070,     3, False, 2),  # 后排第3列 3→区2 仪器(蓝) 3
    # ( 3.375, -0.569,    -3.070,     0, False, 2),  # 后排第3列 0→区3 药品(红) 3

    # ( 3.346, -1.416,    3.075,     2, False, 3),  # 后排第4列 2→区0 食品(绿) 4
    # ( 3.346, -1.416,    3.075,     1, False, 3),  # 后排第4列 1→区1 工具(灰) 4
    # ( 3.346, -1.416,    3.075,     3, False, 3),  # 后排第4列 3→区2 仪器(蓝) 4
    # ( 3.346, -1.416,    3.075,     0, False, 3),  # 后排第4列 0→区3 药品(红) 4

    # ( 3.356, 1.092,     3.075,     2, False, 0),  # 后排第1列 2→区0 食品(绿) 1
    # ( 3.346, 1.323,     3.075,     1, False, 0),  # 后排第1列 1→区1 工具(灰) 1
    # ( 3.689, 1.323,     3.075,     3, False, 0),  # 后排第1列 3→区2 仪器(蓝) 1
    # ( 3.689, 1.323,     3.075,     0, False, 0),  # 后排第1列 0→区3 药品(红) 1

    ( 1.495, -1.319, -0.045, 2, True,  3),  # 前排第4列 2→区3 食品(绿)

    ( 1.525, 1.252, -0.024, 0, True,  0),  # 前排第1列 0→区3 药品(红)
]

# ============================================================
# 4 个归位区点位
# 索引 = 归位区编号: 0=食品(绿), 1=工具(灰), 2=仪器(蓝), 3=药品(红)
# ============================================================
ZONES = [
    # ( 5.429,  1.263,  0.013),  # 区0: 食品 (绿色)
    # ( 5.429,  0.443,  0.010),  # 区1: 工具 (灰色)
    # ( 5.429, -0.352, -0.004),  # 区2: 仪器 (蓝色)
    # ( 5.429, -1.157, -0.001),  # 区3: 药品 (红色)
    ( 4.893, 1.043,  -0.042),  # 区0: 食品 (绿色)
    ( 4.933, 0.237,  -0.023),  # 区1: 工具 (灰色)
    ( 4.927, -0.573, -0.030),  # 区2: 仪器 (蓝色)
    ( 4.866, -1.388, -0.056),  # 区3: 药品 (红色)
   
]

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
    rospy.loginfo("[导航] %s 到达!", label)
    return True


def trigger_arm(pub, label, arm_timeout):
    """发 trigger/place 并等 /arm/done，成功返回 True"""
    rospy.loginfo("[机械臂] %s: 发送触发 → 等待 /arm/done...", label)
    pub.publish(Empty())
    try:
        msg = rospy.wait_for_message('/arm/done', String, timeout=arm_timeout)
        rospy.loginfo("[机械臂] %s: 收到 /arm/done '%s'", label, msg.data)
        return True
    except rospy.ROSException:
        rospy.logerr("[机械臂] %s: 等待超时 (%.0fs)!", label, arm_timeout)
        return False


def delay_before_pick(delay_sec, label):
    if delay_sec <= 0.0:
        return
    rospy.loginfo("[delay] %s: wait %.2fs before arm trigger", label, delay_sec)
    rospy.sleep(delay_sec)


def visual_align_before_pick(target_class, label, service_name, wait_timeout, required, ignore_class):
    align_class = -1 if ignore_class else int(target_class)
    rospy.loginfo("[visual_align] %s: box_type=%d align_class=%d",
                  label, target_class, align_class)
    rospy.set_param("/visual_align/target_class", int(align_class))

    try:
        rospy.wait_for_service(service_name, timeout=wait_timeout)
    except rospy.ROSException:
        rospy.logerr("[visual_align] service %s not available", service_name)
        return not required

    try:
        align_srv = rospy.ServiceProxy(service_name, Trigger)
        resp = align_srv()
    except rospy.ServiceException as e:
        rospy.logerr("[visual_align] service call failed: %s", e)
        return not required

    if resp.success:
        rospy.loginfo("[visual_align] success: %s", resp.message)
        return True

    rospy.logerr("[visual_align] failed: %s", resp.message)
    return not required


def publish_zero_velocity(pub, count, dt):
    zero = Twist()
    for _ in range(max(1, count)):
        pub.publish(zero)
        rospy.sleep(dt)


def backoff_after_place(pub, speed, duration, rate_hz, zero_count, zero_dt):
    if duration <= 0.0:
        return
    hz = max(1.0, rate_hz)
    cmd = Twist()
    cmd.linear.x = -abs(speed)
    rate = rospy.Rate(hz)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    rospy.loginfo("[backoff] place done: reverse %.2fm/s for %.2fs", cmd.linear.x, duration)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        pub.publish(cmd)
        rate.sleep()
    publish_zero_velocity(pub, zero_count, zero_dt)


def stop_navigation_before_visual_align(nav_client, zero_pub, zero_count, zero_dt, sleep_sec):
    nav_client.cancel_goal()
    publish_zero_velocity(zero_pub, zero_count, zero_dt)
    if sleep_sec > 0.0:
        rospy.sleep(sleep_sec)


def main():
    rospy.init_node("nav_goal_sender", anonymous=True)

    # ---- 参数 ----
    # 从第几个箱开始 (0=第一个)
    start_index = rospy.get_param("~start_index", 0)
    # 最多抓几个箱
    max_boxes   = rospy.get_param("~max_boxes", len(BOXES))
    # 单次导航超时 (秒)
    nav_timeout = rospy.get_param("~nav_timeout", 50.0)
    # 单次机械臂操作超时 (秒)
    arm_timeout = rospy.get_param("~arm_timeout", 8.0)
    # 抓取前额外停顿时长 (秒), 等狗站稳再触发机械臂
    pickup_delay = float(rospy.get_param("~pickup_delay", 0.8))
    # ---- 视觉对准 (visual_box_align) ----
    # 是否启用视觉对准
    enable_visual_align = bool(rospy.get_param("~enable_visual_align", True))
    # 对准失败是否跳过当前箱 (True=必须对准成功, False=失败也继续)
    visual_align_required = bool(rospy.get_param("~visual_align_required", False))
    # 抓取前视觉对准是否忽略类别。True=只看位置合适就抓, BOXES里的类型仍用于归位区。
    visual_align_ignore_class = bool(rospy.get_param("~visual_align_ignore_class", True))
    # 视觉对准 Service 名称
    visual_align_service = rospy.get_param("~visual_align_service", "/visual_align/align")
    # 等待 Service 就绪的超时 (秒)
    visual_align_service_wait = float(rospy.get_param("~visual_align_service_wait", 2.0))
    # 导航到达后、对准前先停多久 (秒)
    visual_align_pre_stop_sleep = float(rospy.get_param("~visual_align_pre_stop_sleep", 0.25))
    # 对准前发零速度的话题
    pre_align_zero_cmd_topic = rospy.get_param("~pre_align_zero_cmd_topic", "/raw_cmd_vel")
    # 对准前连发几帧零速度
    pre_align_zero_count = int(rospy.get_param("~pre_align_zero_count", 8))
    # 零速度帧间隔 (秒)
    pre_align_zero_dt = float(rospy.get_param("~pre_align_zero_dt", 0.03))
    # ---- 比赛倒计时 ----
    # 比赛总时长 (秒)
    total_time = float(rospy.get_param("~total_time", 180.0))
    # 预留回出发点的时间 (秒)
    reserve_time = float(rospy.get_param("~reserve_time", 0.2))
    # 出发点坐标 (建图原点)
    start_x = rospy.get_param("~start_x", 0.0)
    start_y = rospy.get_param("~start_y", 0.0)
    start_yaw = rospy.get_param("~start_yaw", 0.0)
    race_start = [None]  # mutable list, 第一个 navigate_to 时赋值

    post_place_backoff_enable = bool(rospy.get_param("~post_place_backoff_enable", True))
    post_place_backoff_speed = float(rospy.get_param("~post_place_backoff_speed", 0.08))
    post_place_backoff_duration = float(rospy.get_param("~post_place_backoff_duration", 4.0))
    post_place_backoff_rate = float(rospy.get_param("~post_place_backoff_rate", 10.0))
    # ---- 确定要跑的箱 ----
    boxes_to_run = BOXES[start_index : start_index + max_boxes]

    rospy.loginfo("=" * 55)
    rospy.loginfo("2026RC 完整比赛流程: 逐箱抓取 → 归位 (4箱策略)")
    rospy.loginfo("  共 %d 个物资箱 (从第 %d 个开始)", len(boxes_to_run), start_index + 1)
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        row_str = "前排" if is_front else "后排"
        side_str = "直达归位区"
        rospy.loginfo("  %d. %s 第%d列 箱(%s) → 区%d %s [%s]",
                      start_index + i + 1, row_str, col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id], side_str)
    rospy.loginfo("=" * 55)

    # ---- 连接 move_base ----
    rospy.loginfo("[初始化] 连接 move_base action server...")
    nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    if not nav_client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr("move_base action server 连接失败!")
        sys.exit(1)
    rospy.loginfo("[初始化] move_base 已连接")

    # ---- 机械臂 publisher ----
    arm_trigger_pub       = rospy.Publisher('/arm/trigger',       Empty, queue_size=1)  # 抓取 pitch=1
    arm_place_pub         = rospy.Publisher('/arm/place',         Empty, queue_size=1)  # 放置1 pitch=4
    arm_heading_zone2_pub = rospy.Publisher('/arm/heading_zone2', Empty, queue_size=1)  # 去区2路上 pitch=7
    arm_place2_pub        = rospy.Publisher('/arm/place2',        Empty, queue_size=1)  # 放置2 pitch=6
    pre_align_zero_pub    = rospy.Publisher(pre_align_zero_cmd_topic, Twist, queue_size=1)
    rospy.sleep(0.3)

    # ---- 归位区访问计数 (每个区最多放2箱) ----
    zone_visit_count = {0: 0, 1: 0, 2: 0, 3: 0}

    # ---- 比赛倒计时辅助 ----
    def _start_race():
        """第一次调用时记录比赛开始时间"""
        if race_start[0] is None:
            race_start[0] = time.time()
            rospy.loginfo("[计时] 比赛开始! 总时长=%.0fs 预留=%.0fs", total_time, reserve_time)

    def _remaining():
        """返回剩余秒数, 未开始时返回无穷大"""
        if race_start[0] is None:
            return float('inf')
        return total_time - (time.time() - race_start[0])

    def _should_go_home():
        """剩余时间 <= 预留值时应立即回出发点"""
        return _remaining() <= reserve_time

    def _go_home(label):
        """取消当前导航, 发布零速度, 导航回出发点"""
        rospy.logwarn("[计时] %s: 只剩 %.0fs, 返回出发点! (%.0f, %.0f)",
                      label, _remaining(), start_x, start_y)
        nav_client.cancel_goal()
        publish_zero_velocity(pre_align_zero_pub, pre_align_zero_count, pre_align_zero_dt)
        ok = navigate_to(nav_client, start_x, start_y, start_yaw, "出发点", nav_timeout)
        if ok:
            rospy.loginfo("[计时] 已回到出发点")
        else:
            rospy.logerr("[计时] 回出发点失败, 但时间已到")
        return True  # 标记已触发回家

    # ============================================================
    # 逐箱循环
    # ============================================================
    success_count = 0
    for i, (bx, by, byaw, btype, is_front, col) in enumerate(boxes_to_run):
        box_num = start_index + i + 1
        zone_id = YOLO_CLASS_TO_ZONE[btype]
        zx, zy, zyaw = ZONES[zone_id]

        rospy.loginfo("=" * 55)
        rospy.loginfo("第 %d/%d 个箱: %s 第%d列 %s → 归位区%d %s",
                      box_num, len(boxes_to_run),
                      "前排" if is_front else "后排", col,
                      BOX_TYPE_NAMES[btype], zone_id, ZONE_NAMES[zone_id])

        # ---- 时间检查 ----
        if _should_go_home() and _go_home("去箱前"):
            break

        # ---- 前排箱: 先经过过渡点1再去找物资箱 ----
        if is_front:
            _start_race()
            if not navigate_to(nav_client, TRANSIT1_X, TRANSIT1_Y, TRANSIT1_YAW,
                               "过渡点1(前排)", nav_timeout):
                rospy.logerr("跳过物资箱 %d (无法到达过渡点1)", box_num)
                continue

        # ---- 去物资箱 → 抓取 ----
        _start_race()
        if not navigate_to(nav_client, bx, by, byaw,
                           "物资箱%d(%s)" % (box_num, BOX_TYPE_NAMES[btype]), nav_timeout):
            rospy.logerr("跳过物资箱 %d (导航失败)", box_num)
            continue

        if _should_go_home() and _go_home("对准前"):
            break

        if enable_visual_align:
            stop_navigation_before_visual_align(
                nav_client,
                pre_align_zero_pub,
                pre_align_zero_count,
                pre_align_zero_dt,
                visual_align_pre_stop_sleep)
            if not visual_align_before_pick(
                    btype,
                    "pick box %d" % box_num,
                    visual_align_service,
                    visual_align_service_wait,
                    visual_align_required,
                    visual_align_ignore_class):
                rospy.logerr("跳过物资箱 %d (视觉对准失败)", box_num)
                continue

        if _should_go_home() and _go_home("抓取前"):
            break

        delay_before_pick(pickup_delay, "pick box %d" % box_num)
        if not trigger_arm(arm_trigger_pub,
                           "抓取箱%d" % box_num, arm_timeout):
            rospy.logerr("跳过物资箱 %d (抓取失败)", box_num)
            continue

        if _should_go_home() and _go_home("放置前"):
            break

        # ---- 去归位区 → 放置 ----
        visit_cnt = zone_visit_count[zone_id]

        if visit_cnt == 0:
            # 第一次去这个区: 路上发0, 到达发 /arm/place (pitch=4)
            place_pub = arm_place_pub
        else:
            # 第二次去这个区: 路上发 pitch=7, 到达发 /arm/place2 (pitch=6)
            rospy.loginfo("  归位区%d 第2次访问 → 路上 pitch=7", zone_id)
            arm_heading_zone2_pub.publish(Empty())
            place_pub = arm_place2_pub

        if not navigate_to(nav_client, zx, zy, zyaw,
                           "归位区%d %s (第%d次)" % (zone_id, ZONE_NAMES[zone_id], visit_cnt + 1),
                           nav_timeout):
            rospy.logerr("箱 %d 归位失败 (导航到归位区失败)", box_num)
            continue

        if not trigger_arm(place_pub,
                           "放置箱%d到区%d" % (box_num, zone_id), arm_timeout):
            rospy.logerr("箱 %d 归位失败 (放置失败)", box_num)
            continue

        if post_place_backoff_enable:
            backoff_after_place(
                pre_align_zero_pub,
                post_place_backoff_speed,
                post_place_backoff_duration,
                post_place_backoff_rate,
                pre_align_zero_count,
                pre_align_zero_dt)

        zone_visit_count[zone_id] += 1
        success_count += 1
        rospy.loginfo("✅ 箱 %d 完成! (区%d 第%d次放置, 总 %d/%d) | 剩余 %.0fs",
                      box_num, zone_id, zone_visit_count[zone_id],
                      success_count, len(boxes_to_run), _remaining())

        # ---- 每个箱完成后检查时间 ----
        if _should_go_home():
            _go_home("箱完成后")
            break

    # ============================================================
    # 回到出发点 (如果还没触发过 _go_home)
    # ============================================================
    if race_start[0] is not None and not _should_go_home():
        rospy.loginfo("[计时] 任务结束, 返回出发点 (剩余 %.0fs)...", _remaining())
        publish_zero_velocity(pre_align_zero_pub, pre_align_zero_count, pre_align_zero_dt)
        navigate_to(nav_client, start_x, start_y, start_yaw, "出发点", nav_timeout)
    elif _should_go_home():
        # 已经触发过 _go_home, 上面已经回去了
        pass

    # ============================================================
    # 完成
    # ============================================================
    rospy.loginfo("=" * 55)
    rospy.loginfo("比赛流程结束!")
    rospy.loginfo("  成功: %d/%d 个箱", success_count, len(boxes_to_run))
    rospy.loginfo("=" * 55)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
