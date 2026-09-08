#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stdint.h>
#include <math.h>

#pragma pack(1)
struct SerialPoseFrame
{
    uint8_t  header1;
    uint8_t  header2;
    int16_t  x;         // pos x, cm (m * 100)
    int16_t  y;         // pos y, cm
    int16_t  z;         // pos z, cm
    int16_t  roll;      // roll, 0.01 deg
    int16_t  pitch;     // pitch, 0.01 deg
    int16_t  yaw;       // yaw, 0.01 deg
    uint8_t  tail;
};
#pragma pack()

class SerialPoseSender
{
public:
    SerialPoseSender()
    {
        std::string serial_port;
        int baudrate;
        double pub_hz;
        nh_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate, 115200);
        nh_.param<double>("pub_hz", pub_hz, 100.0);

        try
        {
            ser_.setPort(serial_port);
            ser_.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(to);
            ser_.open();
            ROS_INFO("Serial opened: %s @ %d", serial_port.c_str(), baudrate);
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("Serial open failed: %s", e.what());
            ros::shutdown();
        }

        odom_sub_ = nh_.subscribe("/Odometry", 1, &SerialPoseSender::poseCallback, this);
        send_timer_ = nh_.createTimer(ros::Duration(1.0 / pub_hz),
                                      &SerialPoseSender::sendTimerCallback, this);
        ROS_INFO("Pose serial sender started at %.0f Hz, y detail tracked", pub_hz);
    }

    ~SerialPoseSender()
    {
        if (ser_.isOpen()) ser_.close();
    }

    void spin()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Timer send_timer_;
    serial::Serial ser_;

    double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0;
    double last_roll_ = 0.0, last_pitch_ = 0.0, last_yaw_ = 0.0;
    bool has_data_ = false;

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        last_x_ = msg->pose.pose.position.x;
        last_y_ = msg->pose.pose.position.y;
        last_z_ = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(last_roll_, last_pitch_, last_yaw_);
        has_data_ = true;
    }

    void sendTimerCallback(const ros::TimerEvent&)
    {
        if (!has_data_) return;

        int16_t x_int = static_cast<int16_t>(round(last_x_ * 100));
        int16_t y_int = static_cast<int16_t>(round(last_y_ * 100));
        int16_t z_int = static_cast<int16_t>(round(last_z_ * 100));
        int16_t roll_int = static_cast<int16_t>(round(last_roll_ * 180.0 / M_PI * 100));
        int16_t pitch_int = static_cast<int16_t>(round(last_pitch_ * 180.0 / M_PI * 100));
        int16_t yaw_int = static_cast<int16_t>(round(last_yaw_ * 180.0 / M_PI * 100));

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

        try
        {
            ser_.write(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
            // print x,y,yaw more frequently for straight-line monitoring
            ROS_INFO_THROTTLE(0.5, "x=%.3fm y=%.3fm(偏差%.1fcm) yaw=%.1fdeg(%.0fcm)",
                              last_x_, last_y_, last_y_ * 100.0,
                              last_yaw_ * 180.0 / M_PI, y_int);
        }
        catch (serial::SerialException& e)
        {
            ROS_WARN_THROTTLE(1.0, "Serial write failed: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_pose_sender_node");
    SerialPoseSender sender;
    sender.spin();
    return 0;
}
