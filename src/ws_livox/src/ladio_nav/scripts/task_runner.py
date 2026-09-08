#!/usr/bin/env python3
"""
task_runner.py — 2026RC 任务赛主控脚本
四足机械狗自动导航 + 机械臂抓取 + 物资箱归位

比赛流程:
  (0) 出发前读取 OCR 智力题答案 → 确定高分归位区编号 (0-3)
  (1) 导航到前排物资箱区，优先抓取高分类型物资箱所在列 (前后两个)
  (2) 每到一个物资箱点位 → 发 /arm/trigger 触发 STM32 抓取 → 等 /arm/done
  (3) 同列两个箱子抓完 → 导航到对应归位区 → 发 /arm/trigger 放箱
  (4) 循环直到时间耗尽或所有箱子处理完毕

用法:
  rosrun my_nav task_runner.py
  rosrun my_nav task_runner.py _strategy:=all      # 抓所有 8 个箱
  rosrun my_nav task_runner.py _strategy:=priority  # 只抓高分类型 (默认)
  rosrun my_nav task_runner.py _dry_run:=true       # 空跑测试：跳过机械臂
"""

import rospy
import sys
import time
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32, Empty, String


# ====================================================================
# 物资箱类型常量
# ====================================================================
BOX_FOOD  = 0   # 食品 (绿色归位区)
BOX_TOOL  = 1   # 工具 (灰色归位区)
BOX_INSTR = 2   # 仪器 (蓝色归位区)
BOX_MED   = 3   # 药品 (红色归位区)

BOX_TYPE_NAMES = ["食品", "工具", "仪器", "药品"]

# ====================================================================
# 场地导航点位 (需实测后填入)
# 坐标格式: (x_m, y_m, yaw_rad, "名称", "描述")
# ====================================================================

# ---- 智力题识别位 ----
WAYPOINT_OCR = (5.0, 2.0, 0.0, "ocr_pos", "智力题识别位 (面对远端显示屏)")

# ---- 前排 4 个物资箱 (靠近减速带一侧, 距入口约 3.5m) ----
# 顺序: 左→右 (0号列 → 3号列)
WAYPOINTS_FRONT_ROW = [
    (1.0, 1.0,  1.57, "box_f_col0", "前排第0列物资箱"),   # 0号列
    (1.6, 1.0,  1.57, "box_f_col1", "前排第1列物资箱"),   # 1号列
    (2.2, 1.0,  1.57, "box_f_col2", "前排第2列物资箱"),   # 2号列
    (2.8, 1.0,  1.57, "box_f_col3", "前排第3列物资箱"),   # 3号列
]

# ---- 后排 4 个物资箱 (靠近入口一侧) ----
WAYPOINTS_BACK_ROW = [
    (1.0, 1.6, -1.57, "box_b_col0", "后排第0列物资箱"),
    (1.6, 1.6, -1.57, "box_b_col1", "后排第1列物资箱"),
    (2.2, 1.6, -1.57, "box_b_col2", "后排第2列物资箱"),
    (2.8, 1.6, -1.57, "box_b_col3", "后排第3列物资箱"),
]

# ---- 4 个归位区 (场地远端靠墙) ----
WAYPOINTS_ZONES = [
    (1.0, 3.6,  0.0, "zone_0", "归位区0 食品(绿)"),
    (1.8, 3.6,  0.0, "zone_1", "归位区1 工具(灰)"),
    (2.6, 3.6,  0.0, "zone_2", "归位区2 仪器(蓝)"),
    (3.4, 3.6,  0.0, "zone_3", "归位区3 药品(红)"),
]

# ---- 减速带通过点 ----
WAYPOINT_SPEED_BUMP = (2.0, 2.5, 0.0, "speed_bump", "减速带通过点")


# ====================================================================
# 工具函数
# ====================================================================

def build_goal(x, y, yaw):
    """构建 MoveBaseGoal"""
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


def status_name(state):
    names = {
        0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED",
        3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED",
        6: "PREEMPTING", 7: "RECALLING", 8: "RECALLED", 9: "LOST",
    }
    return names.get(state, "UNKNOWN")

# ====================================================================
# 任务主控
# ====================================================================

class TaskRunner:
    """比赛任务执行器"""

    def __init__(self):
        rospy.init_node("task_runner", anonymous=True)

        # ---- 参数 ----
        self.strategy = rospy.get_param("~strategy", "priority")  # "all" | "priority"
        self.dry_run = rospy.get_param("~dry_run", False)          # 空跑测试
        self.nav_timeout = rospy.get_param("~nav_timeout", 45.0)   # 单次导航超时 (秒)
        self.arm_timeout = rospy.get_param("~arm_timeout", 25.0)   # 单次抓取超时 (秒)

        # ---- 连接 move_base ----
        rospy.loginfo("Connecting to move_base action server...")
        self._nav_client = actionlib.SimpleActionClient(
            "/move_base1/move_base", MoveBaseAction)
        if not self._nav_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("move_base action server not found!")
            raise RuntimeError("move_base not available")

        # ---- 机械臂控制 ----
        self._arm_trigger_pub = rospy.Publisher('/arm/trigger', Empty, queue_size=1)
        rospy.sleep(0.5)  # 等 publisher 注册

        # ---- OCR 智力题 ----
        self._ocr_answer = None  # Int32: 0-3

        self._log("TaskRunner initialized. strategy=%s dry_run=%s", self.strategy, self.dry_run)

    # ========== 导航接口 ==========

    def navigate_to(self, waypoint, timeout=None):
        """导航到指定点位，返回 True/False"""
        x, y, yaw, name, desc = waypoint
        if timeout is None:
            timeout = self.nav_timeout

        goal = build_goal(x, y, yaw)
        self._log("NAV → %s: (%.2f, %.2f, %.1f°) %s", name, x, y, yaw * 57.3, desc)

        self._nav_client.send_goal(goal)
        finished = self._nav_client.wait_for_result(timeout=rospy.Duration(timeout))

        if not finished:
            self._log_warn("NAV timeout! Cancelling...")
            self._nav_client.cancel_goal()
            return False

        state = self._nav_client.get_state()
        ok = (state == actionlib.GoalStatus.SUCCEEDED)
        self._log("NAV result: %s", "SUCCESS" if ok else "FAILED (%s)" % status_name(state))
        return ok

    # ========== 机械臂接口 ==========

    def trigger_arm(self):
        """触发 STM32 机械臂抓取/放置，等待完成信号"""
        if self.dry_run:
            self._log("DRY RUN: skipping arm trigger")
            rospy.sleep(2.0)
            return True

        # 发触发指令
        self._arm_trigger_pub.publish(Empty())
        self._log("ARM trigger sent → waiting for /arm/done...")

        # 等 STM32 完成信号
        try:
            msg = rospy.wait_for_message('/arm/done', String, timeout=self.arm_timeout)
            if msg is not None:
                self._log("ARM done signal received!")
                return True
        except rospy.ROSException:
            self._log_warn("ARM timeout! No /arm/done received within %.0fs", self.arm_timeout)
            return False

        return True

    # ========== OCR 智力题 ==========

    def wait_for_ocr_answer(self, timeout=30.0):
        """等待 OCR 节点给出答案，返回 0-3 或 None"""
        self._log("Waiting for OCR answer on /ocr_math/answer...")
        try:
            msg = rospy.wait_for_message('/ocr_math/answer', Int32, timeout=rospy.Duration(timeout))
            self._ocr_answer = msg.data
            self._log("OCR answer: %d → 高分归位区: %s", self._ocr_answer,
                       BOX_TYPE_NAMES[self._ocr_answer])
            return self._ocr_answer
        except rospy.ROSException:
            self._log_warn("OCR timeout! No answer received within %.0fs", timeout)
            return None

    # ========== 比赛任务 ==========

    def run(self):
        """执行完整比赛流程"""
        self._log("=" * 50)
        self._log("2026RC 任务赛开始！")
        self._log("=" * 50)

        start_time = time.time()
        TOTAL_TIME = 170  # 180 秒比赛，留 10 秒余量

        # ---- Step 0: 识别智力题 ----
        if not self.navigate_to(WAYPOINT_OCR):
            self._log_err("无法到达 OCR 识别位!")

        answer = self.wait_for_ocr_answer(timeout=15.0)
        if answer is None:
            self._log_err("OCR 识别失败, 默认使用归位区0")
            answer = 0
        self._ocr_answer = answer

        # ---- Step 1: 确定抓取顺序 ----
        # 高分归位区编号对应的物资箱类型 = answer (0=食品,1=工具,2=仪器,3=药品)
        # 策略: 遍历前排 4 列, 找到对应类型的箱
        # TODO: 需要 YOLO 确认哪一列是哪种类型, 目前按预设顺序

        priority_col = answer % 4  # 假设 4 列按 0=食品,1=工具,2=仪器,3=药品 排列
        self._log("高分类型: %s → 优先抓取第 %d 列", BOX_TYPE_NAMES[answer], priority_col)

        # 抓取序列: 优先列的前排 → 优先列的后排 → 其余列
        if self.strategy == "priority":
            cols = [priority_col]
        else:
            # 全部 8 个: 优先列先抓, 然后其余
            cols = [priority_col] + [c for c in range(4) if c != priority_col]

        completed = 0

        for col_idx in cols:
            if time.time() - start_time > TOTAL_TIME:
                self._log("时间不足, 停止任务")
                break

            if completed >= 8:
                break

            # ---- 抓前排物资箱 ----
            self._log("=" * 40)
            self._log("第 %d 列 前排物资箱", col_idx)

            front_wp = WAYPOINTS_FRONT_ROW[col_idx]
            if not self.navigate_to(front_wp):
                self._log_warn("跳过第 %d 列前排", col_idx)
                continue

            if not self.trigger_arm():
                self._log_warn("前排抓取失败, 继续...")

            completed += 1
            self._log("已抓取: %d/8", completed)

            # ---- 抓后排物资箱 ----
            self._log("=" * 40)
            self._log("第 %d 列 后排物资箱", col_idx)

            back_wp = WAYPOINTS_BACK_ROW[col_idx]
            if not self.navigate_to(back_wp):
                self._log_warn("跳过第 %d 列后排", col_idx)
                continue

            if not self.trigger_arm():
                self._log_warn("后排抓取失败, 继续...")

            completed += 1
            self._log("已抓取: %d/8", completed)

            # ---- 去归位区 ----
            self._log("=" * 40)
            self._log("前往归位区 %d (%s)", answer, BOX_TYPE_NAMES[answer])

            zone_wp = WAYPOINTS_ZONES[answer]
            if not self.navigate_to(zone_wp):
                self._log_warn("无法到达归位区!")
                continue

            if not self.trigger_arm():
                self._log_warn("放箱失败, 继续...")

            self._log("归位完成!")
            completed += 2  # 一次放两个箱

        elapsed = time.time() - start_time
        self._log("=" * 50)
        self._log("任务结束! 共抓取 %d 个箱 | 耗时 %.1f 秒", completed, elapsed)
        self._log("=" * 50)

    # ========== 日志 ==========

    @staticmethod
    def _log(msg, *args):
        rospy.loginfo("[TASK] " + msg, *args)

    @staticmethod
    def _log_warn(msg, *args):
        rospy.logwarn("[TASK] " + msg, *args)

    @staticmethod
    def _log_err(msg, *args):
        rospy.logerr("[TASK] " + msg, *args)


# ====================================================================
# 单步测试命令
# ====================================================================

def cmd_line_test():
    """命令行测试模式: 单步执行导航或抓取"""
    rospy.init_node("task_runner_cmd", anonymous=True)

    runner = TaskRunner()

    # 读取命令行参数
    cmd = rospy.get_param("~cmd", "run")  # "run" | "ocr" | "nav" | "arm"

    if cmd == "ocr":
        ans = runner.wait_for_ocr_answer()
        print("OCR Answer: %s" % ans)

    elif cmd == "arm":
        ok = runner.trigger_arm()
        print("ARM result: %s" % ok)

    elif cmd == "nav":
        idx = rospy.get_param("~nav_idx", 0)
        wp = WAYPOINTS_FRONT_ROW[idx]
        ok = runner.navigate_to(wp)
        print("NAV result: %s" % ok)

    else:
        runner.run()


if __name__ == "__main__":
    try:
        cmd_line_test()
    except RuntimeError as e:
        rospy.logerr("Fatal: %s", e)
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
