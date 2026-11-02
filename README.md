yjx_ws2 四足机械狗导航系统 — 建图·定位·导航 全流程教程
平台: Jetson Orin Nano Super + MID360 激光雷达  ROS 版本: Noetic (Python 3)  工程路径: yjx_ws2/livox_ws/src/ws_livox/  Jetson 端路径: /home/lolo/livox_ws2/livox_ws/src/ws_livox/

---
目录
1. 系统架构总览
2. 建图（Mapping）
3. 定位（Localization）
4. 导航（Navigation）
5. 遇到的问题与解决方案
6. 未来工作所需的核心知识
7. 附录：关键文件速查表

---
1. 系统架构总览
1.1 三层架构
整个导航系统分为 感知层 → 定位层 → 规划控制层，各层职责清晰：
┌─────────────────────────────────────────────────────────────┐
│  感知层 (Perception)                                         │
│  livox_ros_driver2 (雷达驱动) → FAST-LIO (里程计+建图)        │
│  输出: /Odometry, /cloud_registered                          │
├─────────────────────────────────────────────────────────────┤
│  定位层 (Localization)                                       │
│  FAST_LIO_LOCALIZATION (ICP 配准 + 位姿融合)                  │
│  输出: /localization (map→body), /scan                       │
├─────────────────────────────────────────────────────────────┤
│  规划控制层 (Planning & Control)                              │
│  my_nav (move_base + TEB/DWA 规划器)                         │
│  输出: /cmd_vel → serial_bridge → STM32 串口                 │
└─────────────────────────────────────────────────────────────┘
1.2 完整数据流
理解数据如何从雷达流向电机，是调试整个系统的基础：
MID360 雷达硬件
    │
    ▼
livox_ros_driver2          ← 雷达驱动，发布原始点云
    │
    ▼
FAST-LIO                   ← 建图/里程计，输出 /Odometry + /cloud_registered
    │
    ▼
global_localization.py     ← ICP 配准，将当前帧与已有地图匹配
    │                           输出 /map_to_odom 变换
    ▼
transform_fusion.py        ← 融合 ICP 结果 + 里程计
    │                           输出 /localization (map→body)
    ▼
Trans_TF_2d.cpp            ← 将 3D 位姿投影为 2D (map→body_2d)
    │
    ▼
pointcloud_to_laserscan    ← 3D 点云转 2D 激光扫描 /scan
    │
    ▼
move_base                  ← 全局+局部规划器 (Dijkstra + TEB)
    │                           输出 /cmd_vel
    ▼
serial_bridge.py           ← 串口桥，打包 16 字节帧发给 STM32
    │
    ▼
STM32                      ← 底层电机控制
1.3 串口通信协议
与 STM32 的通信采用 16 字节定长帧：
暂时无法在飞书文档外展示此内容
调试技巧: 用 serial_simulator.py 可以在不接真机的情况下模拟串口通信，验证上层逻辑是否正确。

---
2. 建图（Mapping）
2.1 建图是什么？为什么需要建图？
建图是让机器人 先"看"一遍环境，生成一张地图，后续定位和导航都依赖这张地图。可以理解为：你去一个新商场，先逛一圈记住布局，之后就能快速找到想去的店。
本工程使用 FAST-LIO 算法进行建图，它能实时将激光雷达的 3D 点云融合成一张高精度的点云地图（PCD 格式），再通过后处理转换为 2D 栅格地图供导航使用。
2.2 建图流程（手把手操作）
第一步：启动建图 Launch
在 Jetson 或 PC 端执行
roslaunch FAST_LIO_LOCALIZATION build_map.launch
这个 launch 文件会同时启动：
- livox_ros_driver2：MID360 雷达驱动
- FAST-LIO：实时 SLAM 算法
第二步：控制机器人遍历环境
建图时需要 手动遥控 机器人走遍场地的每个角落。关键原则：
- 慢速移动：速度太快会导致点云配准失败（FAST-LIO 的帧间匹配依赖运动连续性）
- 覆盖全面：特别是墙角、障碍物边缘等特征丰富的地方要多走几遍
- 回环闭合：尽量走回起点附近，让算法检测到"回到了原来的地方"，这会大幅降低累计误差
第三步：保存地图
建图完成后，会生成两个关键文件：
- PCD 点云地图：完整的 3D 点云，用于 ICP 定位
- 2D 栅格地图：用于 move_base 导航
2.3 从 3D 点云生成 2D 栅格地图
建完 3D 点云地图后，需要转换为 2D 栅格地图。这一步使用 Pointcloud2Map.launch：
roslaunch FAST_LIO Pointcloud2Map.launch
关键参数调整
这是建图中最常需要调的参数，直接影响地图质量：
暂时无法在飞书文档外展示此内容
为什么要滤波？
- min_z = 0.03：地面反射的点云噪声通常在 z=0 附近，切掉 3cm 以下的点可以去噪
- max_z = 0.50：机器人高度约 50cm，头顶以上的障碍物（如天花板、高处横梁）对 2D 导航没有意义，切掉减少干扰
2.4 点云转激光扫描参数
3D 点云还需要转换为 2D 激光扫描（/scan topic），供 move_base 使用：
暂时无法在飞书文档外展示此内容
经验: 这个高度范围需要根据雷达实际安装高度来调。如果雷达装在狗背上方，-0.3 对应的是地面附近，0.5 对应的是头顶上方。

---
3. 定位（Localization）
3.1 定位是什么？
建图之后，机器人需要知道 "我现在在地图的哪个位置"，这就是定位。定位的精度直接决定导航能否成功。
本工程使用 ICP（Iterative Closest Point） 算法进行定位：将实时扫描到的点云与已有地图做配准，算出机器人在地图坐标系下的位姿。
以已有地图为基准，得到机器人在地图坐标系下的位姿
3.2 定位系统组成
定位层由三个核心脚本协作：
global_localization.py    ← 核心：ICP 配准，将当前帧点云与地图匹配
        │
        ▼ 发布 /map_to_odom
transform_fusion.py       ← 融合：将 ICP 结果与 FAST-LIO 里程计融合
        │
        ▼ 发布 /localization (map→body)
Trans_TF_2d.cpp           ← 投影：将 3D 位姿转为 2D TF
        │
        ▼ 发布 TF: map → body_2d
pointcloud_to_laserscan   ← 转换：3D 点云 → 2D 激光 /scan
global_localization.py — ICP 全局定位
这是定位层的核心。它的工作原理：
1. 订阅 FAST-LIO 输出的实时点云 /cloud_registered
2. 加载预先建好的 PCD 点云地图
3. 用 ICP 算法将当前帧与地图对齐，计算变换矩阵
4. 发布 /map_to_odom（地图坐标系到里程计坐标系的变换）
ICP 的核心思想：
- 当前帧点云 = 地图点云 × 旋转矩阵 R + 平移向量 t
- 通过迭代优化 R 和 t，使得当前帧点云与地图点云的匹配误差最小
- 最终得到的 R 和 t 就是机器人的位姿修正量
transform_fusion.py — 位姿融合
ICP 只是定位的一部分。实际使用中需要将 ICP 结果与 FAST-LIO 的里程计融合：
- ICP：绝对定位，但计算频率低（约 1-5Hz），可能偶尔丢失
- 里程计：相对定位，频率高（100Hz+），但有累计漂移
融合策略：ICP 提供"锚点"修正漂移，里程计提供高频插值。
Trans_TF_2d.cpp — TF 投影
将 3D 的 map → body 变换投影为 2D 的 map → body_2d（去掉 roll/pitch，只保留 x, y, yaw）。这一步是因为 move_base 只工作在 2D 平面。
3.3 定位启动
roslaunch FAST_LIO_LOCALIZATION localizetion.launch
注意: launch 文件名是 localizetion.launch（原工程中的拼写），不要写成 localization.launch。
3.4 发送初始位姿
启动定位后，需要告诉机器人 "你在地图的哪个位置出发"。使用 publish_initial_pose.py：
rosrun FAST_LIO_LOCALIZATION publish_initial_pose.py
这会发布一个初始猜测位姿，ICP 会在这个位置附近搜索最佳匹配。如果初始位姿偏差太大，ICP 可能收敛到错误的位置（陷入局部最优）。

---
4. 导航（Navigation）
4.1 导航是什么？
有了地图和定位，导航就是 规划一条从当前位置到目标点的路径，并控制机器人沿着路径走。
本工程使用 ROS 标准的 move_base 框架，配合 TEB 局部规划器。
4.2 move_base 框架
move_base 是 ROS 导航栈的核心节点，包含：
move_base
├── 全局规划器 (Global Planner)
│   └── Dijkstra 算法：在全局代价地图上找到最短路径
├── 局部规划器 (Local Planner)
│   └── TEB (Timed Elastic Band)：实时避障、轨迹优化
├── 全局代价地图 (Global Costmap)
│   └── 基于地图的静态障碍物层
└── 局部代价地图 (Local Costmap)
    └── 基于激光扫描的实时障碍物层
4.3 导航参数配置
4.3.1 代价地图参数 (costmap)
costmap_common_params.yaml — 通用配置：
- obstacle_range：检测障碍物的最大范围
- raytrace_range：光线追踪清除范围
- inflation_radius：障碍物膨胀半径（机器人半径 + 安全余量）
- observation_sources：数据源配置（激光 /scan）
为什么需要膨胀？
机器人不是质点，它有物理尺寸。如果路径刚好贴着障碍物走，机器人会撞上。inflation_radius 就是在障碍物外扩一圈"虚拟边界"，让规划器生成的路径远离障碍物。
4.3.2 全局规划器参数 (global_planner_params.yaml)
- 使用 Dijkstra 算法（比 A* 更稳健，但稍慢）
- allow_unknown: false：不允许规划到未知区域
- default_tolerance：到达目标点的允许误差
4.3.3 局部规划器参数 — TEB
TEB (Timed Elastic Band) 是本工程选用的局部规划器，相比 DWA 的优势：
暂时无法在飞书文档外展示此内容
TEB 关键参数：
- max_vel_x：最大前进速度
- max_vel_theta：最大转向角速度
- acc_lim_x：最大加速度
- min_obstacle_dist：与障碍物最小距离
- inflation_dist：障碍物膨胀距离
4.4 导航启动
一键启动全栈
roslaunch my_nav robot_run.launch
这个 launch 会按顺序启动：
1. 雷达驱动 livox_ros_driver2
2. 定位栈 localizetion.launch
3. 规划控制栈 move_base.launch
单独启动 move_base
roslaunch my_nav move_base.launch
4.5 发送导航目标
方式一：RViz 手动点击
在 RViz 中点击 "2D Nav Goal"，在地图上点选目标位置和朝向。
方式二：脚本发送
单点导航测试
rosrun my_nav nav_goal_sender.py

比赛任务流程
rosrun my_nav task_runner.py
task_runner.py 是比赛任务的主控脚本，串联了完整的 导航→抓取→归位 流程。
4.6 串口桥接
serial_bridge.py 是连接 ROS 和 STM32 的桥梁：
功能：
- 订阅 /cmd_vel（move_base 输出的速度指令）
- 订阅定位结果（机器人的位姿）
- 打包成 16 字节帧，通过串口发送给 STM32
- 接收 STM32 的回读信息（机械臂状态等）
机械臂触发逻辑：
到达目标点后，serial_bridge.py 通过特定的控制字节与 STM32 通信，控制机械臂的抓取和放置动作：
发 1 → STM32 开始抓取，STM32 回 2 表示抓取完成
发 2 → STM32 保持，STM32 回 3 表示可以移动
发 0 或 7 → 移动到归位区（0=第一次去，7=第二次去）
发 4 或 6 → 到达归位区后放置（4=第一次去，6=第二次去）

---
5. 遇到的问题与解决方案
5.1 ARM CPU 过载
问题：Jetson Orin Nano Super 上同时运行雷达驱动、FAST-LIO、定位、导航，CPU 资源紧张。
原因分析：
- FAST-LIO 的点云处理是计算密集型
- ICP 配准需要大量矩阵运算
- move_base + TEB 也是持续计算
解决方案：
- 优化点云滤波参数，减少处理的点数（提高 pointcloud_min_z、降低 pointcloud_max_z）
- 降低不必要的 topic 发布频率
- 使用 nice 或 taskset 给关键进程分配 CPU 优先级
- 考虑将部分计算卸载到 GPU（Jetson 的 CUDA 核心）
5.2 ICP 定位丢失
问题：机器人移动到某些位置时，ICP 突然匹配失败，定位漂移。
原因分析：
- 环境特征不足（如长走廊、空旷区域），ICP 找不到足够的匹配点
- 机器人运动太快，帧间位移过大，超出 ICP 的收敛范围
- 初始位姿偏差太大，陷入局部最优
解决方案：
- 建图时确保特征丰富区域覆盖充分
- 降低机器人最大速度，给 ICP 更多计算时间
- 使用多分辨率 ICP：先粗配准再精配准
- 融合轮式里程计 / IMU 作为 ICP 的初始猜测
5.3 2D 地图质量差
问题：生成的 2D 栅格地图有噪点、缺失、或障碍物变形。
原因分析：
- 点云滤波高度范围设置不合理
- 建图时移动太快导致点云配准误差大
- 地面不平整导致点云高度分布异常
解决方案：
- 调整 pointcloud_min_z / pointcloud_max_z，找到最佳滤波范围
- 建图时慢速、匀速移动
- 多次建图取平均，或手动修复地图（用图片编辑器涂黑/涂白）
5.4 move_base 规划失败
问题：发送导航目标后，move_base 报 "Failed to find a valid plan"。
原因分析：
- 目标点在代价地图中被标记为障碍物
- inflation_radius 设置过大，导致通道被"堵死"
- 全局代价地图与实际环境不匹配
解决方案：
- 在 RViz 中检查代价地图，确认目标点是否可通行
- 减小 inflation_radius（但不能小于机器人半径）
- 更新全局代价地图的静态地图层
5.5 平台迁移问题
问题：工程从 PC 迁移到 Jetson 后，部分功能异常。
原因分析：
- 两个平台的 .bashrc source 顺序不同，导致 ROS_PACKAGE_PATH 优先级冲突
- Jetson 的计算能力有限，某些实时计算可能丢帧
- 串口设备名不同（PC 可能是 /dev/ttyUSB0，Jetson 是 /dev/ttyACM0）
解决方案：
- 统一 .bashrc 中的 source 顺序，确保 livox_ws2 最后 source（优先级最高）
- 在 launch 文件中用参数化方式指定串口设备名
- 使用 rospack find 确认包路径是否正确

---
6. 未来工作所需的核心知识
6.1 ROS 基础（必须掌握）
TF 坐标变换
这是整个导航系统的 骨架。所有传感器数据、位姿信息都通过 TF 关联：
map → odom → body → body_2d → 各传感器
核心概念：
- tf.TransformListener：监听坐标变换
- tf.TransformBroadcaster：发布坐标变换
- tf2：新版 TF 库，推荐使用
调试命令：
查看 TF 树
rosrun tf view_frames
evince frames.pdf

查看两个坐标系之间的变换
rosrun tf tf_echo map body_2d
Topic / Service / Action
暂时无法在飞书文档外展示此内容
Launch 文件
学会写 launch 文件是 ROS 工程管理的基础：
- <include>：嵌套引用其他 launch
- <arg>：参数传递
- <param> / <rosparam>：设置参数
- <remap>：重映射 topic 名称
6.2 SLAM 算法（理解原理）
FAST-LIO 为什么快？
传统 LiDAR SLAM（如 LOAM）需要提取特征点（边缘点+平面点），再做匹配。FAST-LIO 的创新：
1. 直接法：不提取特征，直接用原始点云
2. iKD-Tree：增量式 KD-Tree，加速近邻搜索
3. 迭代卡尔曼滤波：将 IMU 预积分与点云配准融合
ICP 算法流程
输入: 当前帧点云 P, 地图点云 Q
1. 对 P 中每个点，找 Q 中最近的点 → 对应关系
2. 计算最优旋转 R 和平移 t，使误差最小 (SVD 分解)
3. 用 R, t 更新 P
4. 重复 1-3，直到收敛（误差变化 < 阈值）
输出: 变换矩阵 [R|t]
ICP 的局限：
- 需要初始猜测（否则容易陷入局部最优）
- 对动态物体敏感
- 在几何退化场景（如长走廊）效果差
遇到的问题与解决过程
6.3 导航算法（理解原理）
Dijkstra 全局规划
- 在栅格地图上从起点开始，向四周扩展，直到到达目标点
- 保证找到最短路径
- 时间复杂度 O(V²)，用优先队列优化到 O((V+E)logV)
TEB 局部规划
TEB 的核心思想是将路径表示为一系列 带时间戳的位姿点（弹性带），然后优化：
1. 初始化一条从当前位姿到目标的初始轨迹
2. 优化目标函数（包含多项）：
  - 轨迹应该尽可能短
  - 轨迹应该远离障碍物
  - 速度/加速度应该满足约束
  - 轨迹的时间间隔应该合理
3. 输出优化后的轨迹和对应的速度指令
DWA 局部规划（备选）
DWA (Dynamic Window Approach) 的思路不同：
1. 在速度空间中采样 (v, ω)
2. 对每个速度组合，前向模拟一段轨迹
3. 评价每条轨迹（距离障碍物、距离目标、速度）
4. 选择最优轨迹对应的速度
6.4 调试技能（实用）
RViz 可视化
RViz 是 ROS 调试的核心工具，必须熟练使用：
暂时无法在飞书文档外展示此内容
本工程的 RViz 配置文件: my_nav/rviz_cfg/nav_debug.rviz
rosbag 录制与回放
录制所有 topic（调试用）
rosbag record -a -O debug.bag

只录特定 topic
rosbag record /scan /Odometry /localization /cmd_vel -O partial.bag

回放
rosbag play debug.bag --clock

查看 bag 信息
rosbag info debug.bag
常用调试命令
查看 topic 列表
rostopic list

查看 topic 数据
rostopic echo /localization
rostopic echo /cmd_vel

查看 topic 频率
rostopic hz /scan
rostopic hz /Odometry

查看节点关系
rosnode list
rosnode info /move_base

动态调参（运行时修改参数）
rosrun rqt_reconfigure rqt_reconfigure
6.5 机械臂通信协议（STM32）
理解与 STM32 的通信协议对于完成比赛任务至关重要：
抓取流程：
机器人到达物资点
    │
    ▼
发 1 给 STM32（触发抓取）
    │
    ▼
等待 STM32 回 2（抓取完成）
    │
    ▼
发 2 给 STM32（确认）
    │
    ▼
等待 STM32 回 3（可以移动）
    │
    ▼
发 0（第一次去的归位区）或 7（第二次去的归位区）
    │
    ▼
移动到归位区
    │
    ▼
发 4（第一次）或 6（第二次）触发放置
    │
    ▼
等待 STM32 回 2（放置完成）
    │
    ▼
发 2 给 STM32（确认）
    │
    ▼
等待 STM32 回 4（可以移动）
    │
    ▼
发 0（继续下一个任务）

---
7. 附录：关键文件速查表
启动文件
暂时无法在飞书文档外展示此内容
参数文件
暂时无法在飞书文档外展示此内容
脚本文件
暂时无法在飞书文档外展示此内容
调试文件
暂时无法在飞书文档外展示此内容

---
快速排障流程图
导航不工作？
│
├─ 检查 TF 树是否完整
│   └─ rosrun tf view_frames
│      └─ 缺失？→ 检查对应节点是否启动
│
├─ 检查 /scan 是否正常
│   └─ rostopic hz /scan
│      └─ 没数据？→ 检查雷达驱动 + pointcloud_to_laserscan
│
├─ 检查定位是否正常
│   └─ rostopic echo /localization
│      └─ 位姿跳变？→ ICP 丢失，检查初始位姿
│
├─ 检查 move_base 状态
│   └─ rostopic echo /move_base/status
│      └─ ABORTED？→ 检查代价地图 + 目标点是否可达
│
└─ 检查串口通信
    └─ rostopic echo /serial_bridge/debug
       └─ 无数据？→ 检查串口设备名 + 权限

---
最后一句话: 这套系统的每个模块都是可以独立测试的。调试时 从下往上 排查：先确认雷达有数据，再确认定位正常，最后才看导航规划。不要一上来就跑全栈，那只会让你对着一堆报错无从下手。
