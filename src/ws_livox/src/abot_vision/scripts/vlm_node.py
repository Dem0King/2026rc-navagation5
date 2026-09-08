#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
import sys
import re
import os
import subprocess
import random
import base64
import json
import openai
import numpy as np
from collections import Counter
from openai import OpenAI

from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Int32

KEY = ''                         # 大模型APIKEY
API_URL = 'https://ark.cn-beijing.volces.com/api/v3'                 # 大模型API地址
MODEL = 'doubao-seed-2-0-pro-260215'                      # 调用的大模型

# 音频文件目录：对结果取模(0~3)播报 da0~da3 的mp3音频
AUDIO_DIR = os.path.expanduser('/home/lolo/livox_ws2/livox_ws/src/ws_livox/src/abot_vision/audio')

# 识别决策参数
DEADLINE = 20.0     # 启动后必须在该秒数内完成识别决策
NEED = 1            # 连续采集的结果个数：全同则播该结果，全不同则随机

# 调用大模型前的助手提示词
PROMPT = """
图像识别系统规则

一、识别规则
1. 前置校验
- 仅当图像中存在智力题目显示区，且题目以 Times New Roman 字体清晰呈现时，才进入计算流程。
- 若字体非 Times New Roman、字体无法识别或无题目显示区 → 直接输出 None。

2. 算式解析与计算
- 题目必须为一道四则运算表达式（含 + − × ÷），所有参与运算的数必须为整数。
- 严格遵循数学运算优先级（先乘除后加减，括号优先）进行计算。
- 最终计算结果必须为整数；若结果为小数、分数或出现除零等数学错误 → 输出 None。

3. 输出格式
- 仅输出最终计算结果的阿拉伯数字（例如：24）。
- 禁止包含任何文字、符号、空格、换行或推理过程。
- 不满足上述任一条件 → 输出 None。

二、示例集
| 输入题目（Times New Roman字体） | 输出 | 说明 |
| :--- | :--- | :--- |
| 3 + 5 | 8 | 简单加法，结果为整数 |
| 12 − 4 × 2 | 4 | 含乘减混合运算，遵循优先级 |
| (18 + 6) ÷ 3 − 2 | 6 | 含括号与多步运算，结果为整数 |
| 7 + 3.5 | None | 含小数，不符合整数运算规则 |
"""

###################################################################################################

# 最近一帧图像缓存（回调只负责更新，主循环按时间戳取用识别）
latest_frame = None

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

    # YUYV (yuv422) 是单通道排布的 2 字节/像素格式，需要单独处理
    if img_msg.encoding in ("yuyv", "yuv422_yuy2"):
        yuyv = np.ndarray(shape=(img_msg.height, img_msg.width, 2), dtype=dtype, buffer=img_msg.data)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)

    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # If the byte order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Convert to BGR if the encoding is not already BGR
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv


SAVE_PATH = os.path.expanduser('/home/lolo/livox_ws2/livox_ws/src/ws_livox/src/abot_vision/temp/vl_now.jpg')


def yi_vision_api(_PROMPT=PROMPT, img_path=SAVE_PATH):

    client = OpenAI(
        api_key = KEY,
        base_url = API_URL
    )

    # 编码为base64数据
    with open(img_path, 'rb') as image_file:
        image = 'data:image/jpeg;base64,' + base64.b64encode(image_file.read()).decode('utf-8')

    # 向大模型发起请求
    completion = client.chat.completions.create(
        model = MODEL,
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": _PROMPT},
                    {"type": "image_url", "image_url": {"url": image}}
                ]
            }
        ]
    )

    # 解析大模型返回结果
    result_str = completion.choices[0].message.content.strip()
    result = str(result_str)

    return result

def play_audio(idx):
    '''播报 da{idx}.mp3 (idx 已是 0~3 的分区编号)'''
    audio_path = os.path.join(AUDIO_DIR, f'da{idx}.mp3')
    if not os.path.exists(audio_path):
        rospy.logerr('音频文件不存在: %s', audio_path)
        return
    try:
        # mpg123 播放 mp3；-q 静默，阻塞播放，播完返回
        subprocess.run(['mpg123', '-q', audio_path], check=True)
        rospy.loginfo('播放音频: %s', audio_path)
    except Exception as e:
        rospy.logerr('音频播放失败: %s', e)


def recognize_once():
    '''取最近一帧 → 存图 → 调大模型 → 返回识别数字；失败返回None'''
    img_bgr = latest_frame
    if img_bgr is None:
        return None

    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
    if not cv2.imwrite(SAVE_PATH, img_bgr):
        rospy.logerr('图片保存失败: %s', SAVE_PATH)
        return None

    try:
        res = yi_vision_api()
    except Exception as e:
        rospy.logerr('API调用异常: %s', e)
        return None

    numbers = re.findall(r'[0-9]+', res)
    data = int(numbers[-1]) if numbers else None
    rospy.loginfo('单次识别: 原始=%s 结果=%s', res, data)
    return data


def decide(results):
    '''
    根据采集到的结果列表决定要播报的分区编号(0~3)：
    - 凑满3个：全同→该结果；全不同→随机；部分一致→多数票
    - 不足3个(超时)：取出现次数最多的(并列随机)；一个都没有→随机
    '''
    if not results:
        rospy.logwarn('未取得任何有效结果，随机播报')
        return random.randint(0, 3)

    counter = Counter(results)
    top = counter.most_common(1)[0][1]          # 最高出现次数

    if len(results) >= NEED and top == 1:
        # 三个互不相同 → 随机
        choice = random.choice(results)
        rospy.loginfo('连续%d次结果全不同%s，随机选 %s', NEED, results, choice)
    else:
        # 全同 / 多数票 / 超时取众数；并列时随机挑一个
        winners = [v for v, c in counter.items() if c == top]
        choice = random.choice(winners)
        rospy.loginfo('结果%s，按多数票/众数选 %s', results, choice)

    return choice % 4                           # 对4取模得到分区编号


def top_view_shot(image_msg):
    '''图像回调：只缓存最新帧，识别与播报由主循环负责'''
    global latest_frame
    img_bgr = imgmsg_to_cv2(image_msg)
    if img_bgr is not None:
        latest_frame = img_bgr


def main():
    rospy.init_node('abot_vlm_node')
    answer_pub = rospy.Publisher('/ocr_math/answer', Int32, queue_size=1, latch=True)
    value_pub = rospy.Publisher('/ocr_math/value', Int32, queue_size=1, latch=True)
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot, queue_size=1, buff_size=2**24)
    rospy.loginfo('视觉大模型节点启动，%.0f秒内完成识别决策...', DEADLINE)

    deadline = time.time() + DEADLINE
    results = []

    # 等首帧到达（受总截止时间约束）
    while latest_frame is None and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.05)

    # 在截止时间内连续识别，凑满NEED个或超时为止
    while len(results) < NEED and time.time() < deadline and not rospy.is_shutdown():
        data = recognize_once()
        if data is not None:
            results.append(data)

    # 决策并播报
    idx = decide(results)
    raw_value = int(results[-1]) if results else idx
    answer_pub.publish(Int32(idx))
    value_pub.publish(Int32(raw_value))
    rospy.set_param('/abot_vlm_node/im_result', idx)
    rospy.set_param('/abot_vlm_node/im_flag_memory_time', time.time())
    rospy.loginfo('最终分区编号: %s', idx)
    play_audio(idx)

    # 播报完成后自动结束本进程
    rospy.loginfo('播报完成，退出节点进程')
    rospy.signal_shutdown('播报完成')
    os._exit(0)

if __name__ == '__main__':
    main()
