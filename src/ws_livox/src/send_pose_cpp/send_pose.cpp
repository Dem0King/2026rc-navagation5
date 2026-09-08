#include <ros/ros.h>                // ROS C++ 核心库，对应 rospy
#include <serial/serial.h>          // ROS 官方串口库，对应 pyserial
#include <nav_msgs/Odometry.h>      // 里程计消息类型，对应 from nav_msgs.msg import Odometry
#include <tf2/LinearMath/Quaternion.h>  // 四元数运算
#include <tf2/LinearMath/Matrix3x3.h>   // 四元数转欧拉角
#include <stdint.h>                 // 标准整数类型，int16_t/uint8_t 等
#include <math.h>                   // 数学函数，对应 math.degrees

// ============== 串口帧协议定义 ==============
// 对应 Python 里的帧格式注释 + struct.pack
// #pragma pack(1) 是关键：取消结构体的字节对齐，保证内存布局和协议完全一致
// 就像 Python struct.pack 是严格按顺序拼字节，C++ 默认会有空字节填充，必须加这个取消
#pragma pack(1)
struct SerialPoseFrame
{
    uint8_t  header1;  
    uint8_t  header2;  
    int16_t  x;         // 位置x，单位cm (m * 100)
    int16_t  y;         // 位置y，单位cm
    int16_t  z;         // 位置z，单位cm
    int16_t  roll;      // 横滚角，单位0.01° (° * 100)
    int16_t  pitch;     // 俯仰角，单位0.01°
    int16_t  yaw;       // 偏航角，单位0.01°
    uint8_t  tail;      
};
#pragma pack() // 恢复默认字节对齐，避免影响后面的代码

// ============== 核心类 ==============
// 对应 Python 的 class LocalizationToSerial
class SerialPoseSender
{
public:
    // 构造函数：初始化节点、打开串口、订阅话题
    // 对应 Python 的 __init__
    SerialPoseSender()
    {
        // 1. 读取 ROS 参数（对应 Python 的 rospy.get_param）
        // 好处：不用改代码重新编译，启动时传参就能改串口名、波特率
        std::string serial_port;
        int baudrate;
        nh_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate, 115200);

        // 2. 打开串口（对应 Python 的 serial.Serial）
        try
        {
            ser_.setPort(serial_port);          // 设置串口设备名
            ser_.setBaudrate(baudrate);         // 设置波特率
            // 串口配置：8位数据位、无校验、1位停止位（8N1，和Python默认一致）
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(to);
            ser_.open(); // 打开串口
            ROS_INFO("串口打开成功: %s, 波特率: %d", serial_port.c_str(), baudrate);
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("串口打开失败: %s", e.what());
            ros::shutdown(); // 串口打不开直接退出，和Python逻辑一致
        }

        // 3. 订阅里程计话题（对应 Python 的 rospy.Subscriber）
        // queue_size=1：只保留最新一帧，降低延迟，对应 Python 的 queue_size=1
odom_sub_ = nh_.subscribe("/Odometry", 1, &SerialPoseSender::poseCallback, this);

        ROS_INFO("位姿串口发送节点启动完成");
    }

    // 析构函数：节点退出时自动关闭串口
    ~SerialPoseSender()
    {
        if (ser_.isOpen())
        {
            ser_.close();
        }
    }

    // 主循环，对应 Python 的 spin()
    void spin()
    {
        ros::spin(); // 等待回调触发，和 rospy.spin() 完全一样
    }

private:
    ros::NodeHandle nh_;       // ROS 节点句柄，相当于 Python 的节点本体
    ros::Subscriber odom_sub_;  // 订阅者对象
    serial::Serial ser_;        // 串口对象

    // ============== 里程计回调函数 ==============
    // 对应 Python 的 pose_callback，功能完全一致
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 1. 取出位置和四元数（对应 Python 的 pos = msg.pose.pose.position）
        double pos_x = msg->pose.pose.position.x;
        double pos_y = msg->pose.pose.position.y;
        double pos_z = msg->pose.pose.position.z;

        // 2. 四元数转欧拉角（对应 Python 的 tf_trans.euler_from_quaternion）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 得到的是弧度，和Python输出一致

        // 3. 定点量化：转成 int16（和 Python 逻辑完全一样）
        // 位置：米 → 厘米，乘100四舍五入
        int16_t x_int = static_cast<int16_t>(round(pos_x * 100));
        int16_t y_int = static_cast<int16_t>(round(pos_y * 100));
        int16_t z_int = static_cast<int16_t>(round(pos_z * 100));

        // 姿态：弧度 → 度 → 0.01度，乘100四舍五入
        int16_t roll_int = static_cast<int16_t>(round(roll * 180.0 / M_PI * 100));
        int16_t pitch_int = static_cast<int16_t>(round(pitch * 180.0 / M_PI * 100));
        int16_t yaw_int = static_cast<int16_t>(round(yaw * 180.0 / M_PI * 100));

        // 4. 组装串口帧（对应 Python 的 struct.pack 拼字节）
        SerialPoseFrame frame;
        frame.header1 = 0xaa;
        frame.header2 = 0x55;
        frame.x = x_int;
        frame.y = y_int;
        frame.z = z_int;
        frame.roll = roll_int;
        frame.pitch = pitch_int;
        frame.yaw = yaw_int;
        frame.tail = 0xff;

        // 5. 串口发送（对应 Python 的 ser.write）
        try
        {
            // 把结构体指针转成字节指针，直接发送内存里的原始字节
            // 因为我们加了 #pragma pack(1)，内存布局和协议完全匹配
            ser_.write(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));

            // 限频日志，每秒打印一次，对应 Python 的 rospy.loginfo_throttle(1, ...)
            ROS_INFO_THROTTLE(1.0, "发送: x=%d y=%d yaw=%d", x_int, y_int, yaw_int);
        }
        catch (serial::SerialException& e)
        {
            ROS_WARN_THROTTLE(1.0, "串口发送失败: %s", e.what());
        }
    }
};

// ============== 主函数 ==============
// 对应 Python 的 if __name__ == "__main__"
int main(int argc, char** argv)
{
    // 初始化 ROS 节点，对应 rospy.init_node
    ros::init(argc, argv, "serial_pose_sender_node");

    // 创建核心对象，自动执行构造函数
    SerialPoseSender sender;

    // 进入循环等待回调
    sender.spin();

    return 0;
}