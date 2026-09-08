#!/usr/bin/env python3
"""
serial_bridge.py
订阅 /cmd_vel (速度指令) 和 /localization (定位姿态)，打包为 15 字节帧通过串口发送给狗端 STM32。

帧格式 (15 bytes):
  Header:  AA 55        (2 bytes)
  Data:    6 x int16 LE (12 bytes)
           [0] vx      = m/s * 100
           [1] vy      = m/s * 100
           [2] ang_z   = rad/s * 100
           [3] yaw     = degrees * 100
           [4] roll    = degrees * 100
           [5] pitch   = degrees * 100
  Tail:    FF           (1 byte)

注意: 机械臂的触发/回读在 RC_viewer 的 stm32_track_bridge_phycical.py 中处理
"""
import rospy
import serial
import struct
import math
import threading
import tf.transformations as tf_trans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class SerialBridge:
    def __init__(self):
        rospy.init_node('serial_bridge', anonymous=True)

        # port = rospy.get_param('~port', '/dev/ttyCH343USB0')
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        self.ser = serial.Serial(port, baud, timeout=1)

        self._lock = threading.Lock()
        self._cmdvel = None
        self._yaw = 0.0
        self._roll = 0.0
        self._pitch = 0.0

        rospy.Subscriber('/localization', Odometry, self._pose_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self._cmdvel_cb, queue_size=1)

        # 50 Hz 定时发送
        rospy.Timer(rospy.Duration(0.02), self._send)

        rospy.loginfo("serial_bridge: listening /cmd_vel + /localization -> %s @ %d",
                      port, baud)

    def _pose_cb(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        r, p, y = tf_trans.euler_from_quaternion(q)
        with self._lock:
            self._roll = math.degrees(r)
            self._pitch = math.degrees(p)
            self._yaw = math.degrees(y)

    def _cmdvel_cb(self, msg):
        with self._lock:
            self._cmdvel = msg

    def _send(self, event):
        with self._lock:
            if self._cmdvel is None:
                return
            vx = self._cmdvel.linear.x
            vy = self._cmdvel.linear.y
            ang = self._cmdvel.angular.z
            yaw = self._yaw
            roll = self._roll
            pitch = self._pitch

        data = (
            struct.pack('<B', 0xAA) +
            struct.pack('<B', 0x55) +
            struct.pack('<h', int(round(vx * 100))) +
            struct.pack('<h', int(round(vy * 100))) +
            struct.pack('<h', int(round(ang * 100))) +
            struct.pack('<h', int(round(yaw * 100))) +
            struct.pack('<h', int(round(roll * 100))) +
            struct.pack('<h', int(round(pitch * 100))) +
            struct.pack('<B', 0xFF)
        )

        try:
            self.ser.write(data)
            rospy.loginfo_throttle(2,
                "vx=%.2f vy=%.2f ang=%.2f | yaw=%.1f roll=%.1f pitch=%.1f",
                vx, vy, ang, yaw, roll, pitch)
        except serial.SerialException as e:
            rospy.logwarn("serial write error: %s", e)


if __name__ == '__main__':
    try:
        SerialBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
