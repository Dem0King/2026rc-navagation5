#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse


class VisualBoxAlign(object):
    def __init__(self):
        rospy.init_node("visual_box_align", anonymous=False)

        # 订阅 YOLO 检测结果的话题
        self.track_topic = rospy.get_param("~track_topic", "/yolo_depth/track_control")
        # 发布速度控制的话题 (发给 move_base 或直接给串口)
        self.cmd_topic = rospy.get_param("~cmd_topic", "/raw_cmd_vel")
        # 触发对准的 ROS Service 名称
        self.service_name = rospy.get_param("~service_name", "/visual_align/align")

        # 目标纵深距离 (m), 狗对准后离箱子的理想距离
        self.target_x = float(rospy.get_param("~target_x", 0.33))
        # 纵深容差 (m), |实际-目标| < 此值即视为对准
        self.x_tolerance = float(rospy.get_param("~x_tolerance", 0.03))
        self.target_y = float(rospy.get_param("~target_y", 0.0)) # 0.0
        self.y_tolerance = float(rospy.get_param("~y_tolerance", 0.03))
        # 稳定判定: 连续多少帧在容差内
        self.settle_frames = int(rospy.get_param("~settle_frames", 6))
        # 稳定判定: 在容差内持续多少秒
        self.settle_time = float(rospy.get_param("~settle_time", 0.5))
        # 对准总超时 (秒)
        self.timeout = float(rospy.get_param("~timeout", 12.0)) # 8
        # YOLO 数据过期时间 (秒), 超过此值认为丢失目标
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 1.0))

        # P 控制器频率 (Hz)
        self.control_rate = float(rospy.get_param("~control_rate", 15.0))
        # P 控制器比例增益: vx = kp_x × (当前x - 目标x)
        self.kp_x = float(rospy.get_param("~kp_x", 0.35))
        # 最小前向速度 (m/s), 防止速度太小狗不动
        self.min_vx = float(rospy.get_param("~min_vx", 0.045))
        # 最大前向速度 (m/s), 防止冲太快
        self.max_vx = float(rospy.get_param("~max_vx", 0.12))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 1.2))
        self.min_wz = float(rospy.get_param("~min_wz", 0.08))
        self.max_wz = float(rospy.get_param("~max_wz", 0.25))
        self.yaw_sign = float(rospy.get_param("~yaw_sign", -1.0))

        # 是否校验 YOLO 类别 (True=只对指定类别对准)。比赛点位已写死时建议 False，
        # 避免同一个箱类别跳变导致已经对准却不抓取。
        self.check_class = bool(rospy.get_param("~check_class", False))
        # 水平方向限制 (m), |y| > 此值视为偏离太远, 0=不限制
        self.y_abs_limit = float(rospy.get_param("~y_abs_limit", 0.0))
        # 停止时发零速度的帧数 (确保狗停下来)
        self.zero_burst_count = int(rospy.get_param("~zero_burst_count", 8))
        # 零速度帧之间的间隔 (秒)
        self.zero_burst_dt = float(rospy.get_param("~zero_burst_dt", 0.03))
        # 对准完成后的额外停留时间 (秒)
        self.post_stop_sleep = float(rospy.get_param("~post_stop_sleep", 0.25))

        self._lock = threading.Lock()
        self._active_lock = threading.Lock()
        self._latest = None
        self._latest_stamp = rospy.Time(0)

        self._cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.track_topic, Float32MultiArray, self._track_cb,
                         queue_size=1, tcp_nodelay=True)
        self._srv = rospy.Service(self.service_name, Trigger, self._align_cb)
        rospy.on_shutdown(self._publish_zero_burst)

        rospy.loginfo(
            "visual_box_align ready: track=%s cmd=%s target_x=%.3f target_y=%.3f tol_x=%.3f tol_y=%.3f service=%s",
            self.track_topic, self.cmd_topic, self.target_x, self.target_y,
            self.x_tolerance, self.y_tolerance, self.service_name
        )

    def _track_cb(self, msg):
        if len(msg.data) < 8:
            return
        with self._lock:
            self._latest = list(msg.data)
            self._latest_stamp = rospy.Time.now()

    def _get_latest(self):
        with self._lock:
            if self._latest is None:
                return None, rospy.Time(0)
            return list(self._latest), self._latest_stamp

    def _target_class(self):
        if rospy.has_param("/visual_align/target_class"):
            return int(rospy.get_param("/visual_align/target_class"))
        return int(rospy.get_param("~target_class", -1))

    def _publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self._cmd_pub.publish(msg)

    def _publish_vx(self, vx):
        self._publish_cmd(vx, 0.0)

    def _publish_zero_burst(self):
        """连发多帧零速度, 确保狗彻底停下来 (突发模式, 防止丢帧)"""
        zero = Twist()
        for _ in range(max(1, self.zero_burst_count)):
            self._cmd_pub.publish(zero)
            rospy.sleep(self.zero_burst_dt)

    def _valid_track(self, data, stamp, target_class):
        """校验 YOLO 检测数据: 新鲜? 有效? 类别对? Y偏移不超限?"""
        if data is None:
            return False, "no_track", 0.0, 0.0, -1
        if (rospy.Time.now() - stamp).to_sec() > self.stale_timeout:
            return False, "stale_track", 0.0, 0.0, -1

        x = float(data[2])
        y = float(data[3])
        valid = int(round(float(data[6]))) != 0
        cls_id = int(round(float(data[7])))

        if not valid or x <= 0.0:
            return False, "invalid_track", x, y, cls_id
        if self.check_class and target_class >= 0 and cls_id != target_class:
            return False, "wrong_class_%d" % cls_id, x, y, cls_id
        if self.y_abs_limit > 0.0 and abs(y) > self.y_abs_limit:
            return False, "y_out_%.3f" % y, x, y, cls_id
        return True, "ok", x, y, cls_id

    def _align_cb(self, _req):
        if not self._active_lock.acquire(False):
            return TriggerResponse(success=False, message="visual alignment already running")

        try:
            rospy.set_param("/visual_align/active", True)
            return self._run_align()
        finally:
            self._publish_zero_burst()
            rospy.set_param("/visual_align/active", False)
            self._active_lock.release()

    def _run_align(self):
        """对准主循环: P控制器驱动狗前后挪动, 直到 x 进入容差并稳定"""
        target_class = self._target_class()
        rate = rospy.Rate(max(1.0, self.control_rate))
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        stable_count = 0
        stable_since = None
        last_reason = "not_started"

        rospy.loginfo(
            "visual align start: target_class=%d target_x=%.3f target_y=%.3f tol_x=%.3f tol_y=%.3f",
            target_class, self.target_x, self.target_y, self.x_tolerance, self.y_tolerance
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            data, stamp = self._get_latest()
            ok, reason, x, y, cls_id = self._valid_track(data, stamp, target_class)
            last_reason = reason

            if not ok:
                stable_count = 0
                stable_since = None
                self._publish_vx(0.0)
                rospy.logwarn_throttle(
                    0.8, "visual align waiting: %s x=%.3f y=%.3f cls=%d",
                    reason, x, y, cls_id
                )
                rate.sleep()
                continue

            err_x = x - self.target_x
            err_y = y - self.target_y

            if abs(err_y) > self.y_tolerance:
                stable_count = 0
                stable_since = None
                wz = self.yaw_sign * self.kp_yaw * err_y
                wz = max(-self.max_wz, min(self.max_wz, wz))
                if abs(wz) < self.min_wz:
                    wz = math.copysign(self.min_wz, wz)
                self._publish_cmd(0.0, wz)
                rospy.loginfo_throttle(
                    0.5, "visual align yaw: y=%.3f target=%.3f err_y=%.3f wz=%.3f x=%.3f cls=%d",
                    y, self.target_y, err_y, wz, x, cls_id
                )
            elif abs(err_x) <= self.x_tolerance:
                if stable_since is None:
                    stable_since = rospy.Time.now()
                stable_count += 1
                stable_duration = (rospy.Time.now() - stable_since).to_sec()
                self._publish_cmd(0.0, 0.0)
                rospy.loginfo_throttle(
                    0.5, "visual align stable %d/%d %.2fs/%.2fs: x=%.3f err_x=%.3f y=%.3f err_y=%.3f cls=%d",
                    stable_count, self.settle_frames, stable_duration, self.settle_time,
                    x, err_x, y, err_y, cls_id
                )
                if stable_count >= self.settle_frames and stable_duration >= self.settle_time:
                    self._publish_zero_burst()
                    rospy.sleep(self.post_stop_sleep)
                    msg = "aligned x=%.3f err_x=%.3f y=%.3f err_y=%.3f cls=%d" % (x, err_x, y, err_y, cls_id)
                    rospy.loginfo("visual align done: %s", msg)
                    return TriggerResponse(success=True, message=msg)
            else:
                stable_count = 0
                stable_since = None
                vx = self.kp_x * err_x
                vx = max(-self.max_vx, min(self.max_vx, vx))
                if abs(vx) < self.min_vx:
                    vx = math.copysign(self.min_vx, vx)
                self._publish_vx(vx)
                rospy.loginfo_throttle(
                    0.5, "visual align move: x=%.3f target=%.3f err_x=%.3f vx=%.3f y=%.3f err_y=%.3f cls=%d",
                    x, self.target_x, err_x, vx, y, err_y, cls_id
                )

            rate.sleep()

        self._publish_zero_burst()
        msg = "timeout, last_reason=%s" % last_reason
        rospy.logwarn("visual align failed: %s", msg)
        return TriggerResponse(success=False, message=msg)


if __name__ == "__main__":
    try:
        VisualBoxAlign()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
