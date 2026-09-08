#!/usr/bin/env python3
"""
将定位里程计(/localization, nav_msgs/Odometry)通过串口发送给下位机。

数据来源与用途
- 上游通常由定位/融合节点发布 `/localization`（本工程里对应 `fast_lio_localization/scripts/transform_fusion.py`）。
- 本脚本把 ROS 里的 3D 位姿(位置 + 四元数)转换为欧拉角，并按自定义二进制帧格式经串口输出。

串口帧格式（小端）
- 帧头: 0xFF 0xFE
- 负载: x,y,z,roll,pitch,yaw 共 6 个 int16（`<h`，-32768~32767）
  - x/y/z: 位置 *100（m -> cm，再四舍五入）
  - roll/pitch/yaw: 角度(°) *100（deg -> 0.01deg，再四舍五入）
- 帧尾: 0xFD

注意事项
- 量化与溢出: 由于打包为 int16，数值超过范围会在 struct.pack 时抛异常或溢出风险， 
  例如位置超过约 327.67m 或角度超过约 327.67°（乘 100 后超界）。
- 设备名: 默认写死 `/dev/ttyS0`（更像是 Linux 串口设备路径）；在 Windows/不同平台需改为对应串口名。
"""
import rospy
import serial
import struct
import math
import tf.transformations as tf_trans
from nav_msgs.msg import Odometry

class LocalizationToSerial:
    def __init__(self):
        # 初始化 ROS 节点：负责把定位话题转为串口数据
        rospy.init_node('localization_to_serial_node', anonymous=True)
        # 打开串口：115200, 8N1（pyserial 默认），timeout=1s
        # self.ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        self.ser = serial.Serial('/dev/ttyS7', 115200, timeout=1)
        # 订阅融合定位输出（nav_msgs/Odometry），队列设为 1 以降低延迟
        rospy.Subscriber('/localization', Odometry, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        # 从 Odometry 中取出位置与姿态（四元数）
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # 四元数 -> 欧拉角（roll/pitch/yaw，单位: rad）
        q = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(q)

        # 乘 100 后四舍五入为 16-bit 整数
        # - 位置: m -> cm
        # - 姿态: rad -> deg -> 0.01deg
        x = int(round(pos.x * 100))
        y = int(round(pos.y * 100))
        z = int(round(pos.z * 100))
        r = int(round(math.degrees(roll)  * 100))
        p = int(round(math.degrees(pitch) * 100))
        yw = int(round(math.degrees(yaw)  * 100))

        # 按自定义协议拼帧（小端）。int16 用 `<h`。
        data = (
            struct.pack('<B', 0xff) +
            struct.pack('<B', 0xfe) +
            struct.pack('<h', x) +
            struct.pack('<h', y) +
            struct.pack('<h', z) +
            struct.pack('<h', r) +
            struct.pack('<h', p) +
            struct.pack('<h', yw) +
            struct.pack('<B', 0xfd)
        )
        try:
            # 发送到下位机/外设
            self.ser.write(data)
            # 限频日志，避免刷屏（这里打印的是量化后的整数）
            rospy.loginfo_throttle(1, f"Sent: x={x} y={y} yaw={yw}")
        except serial.SerialException as e:
            # 串口写失败时提示，但不让节点直接崩溃
            rospy.logwarn(f"Serial write failed: {e}")

    def spin(self):
        rospy.loginfo("localization_to_serial_node running ...")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LocalizationToSerial()
        node.spin()
    except rospy.ROSInterruptException:
        pass

