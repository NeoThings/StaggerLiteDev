#ifndef STAGGERLITE_CORE_H
#define STAGGERLITE_CORE_H

#include <thread>
#include <ros/ros.h>

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <serial/serial.h>

#define STX 0x55AA;

class StaggerLiteCore {
public:
  StaggerLiteCore(ros::NodeHandle& nh);
  ~StaggerLiteCore();

public:
  void ReadSerial();
  void WriteSerial();

public:
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;

private:
  tf::TransformBroadcaster odom_tf_;

private:
  std::unique_ptr<serial::Serial> stm32_serial_;
  std::string port_;
  int baud_;

private:
  bool brake_;

private:
  const float ODOM_POSE_COV[36] = {1e-3, 0, 0, 0, 0, 0,
                                   0, 1e-3, 0, 0, 0, 0,
                                   0, 0, 1e6, 0, 0, 0,
                                   0, 0, 0, 1e6, 0, 0,
                                   0, 0, 0, 0, 1e6, 0,
                                   0, 0, 0, 0, 0, 1e-3};

private:
  void handleCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

private:
  uint16_t swap16(uint16_t x);
  uint32_t swap32(uint32_t x);

private:
  typedef struct __attribute((packed)){
    uint16_t header;
    float linear_vel;
    float angular_vel;
    bool brake;
    uint8_t crc8;
  }CmdData;

  typedef struct __attribute((packed)){
    uint16_t header;
    float odom_linear_vel;
    float odom_angular_vel; 
    float odom_x;
    float odom_y;
    float odom_theta;
    uint8_t crc8;
  }OdomData;

  typedef union{
    CmdData data;
    uint8_t raw_data[sizeof(data)];
  }cmd_union;

  typedef union{
    OdomData data;
    uint8_t raw_data[sizeof(data)];
  }odom_union;

};

#endif