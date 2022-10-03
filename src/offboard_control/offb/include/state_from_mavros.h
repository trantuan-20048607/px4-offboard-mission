/**
 * state_from_mavros.h
 *
 * @author Qyp
 * @date 2019.6.29
 *
 * @details 本文件中的函数主要用于连接 px4_command 与 mavros 两个功能包,
 *   订阅 mavros 发布的飞控状态量, 包括无人机状态, 位置, 速度, 角度, 角速度.
 * @note 这里并没有订阅所有可以来自飞控的消息.
 *   如需其他消息, 请参阅 mavros 代码.
 *   代码中, 参与运算的角度均是以 rad 为单位,
 *   但是涉及到显示时或者需要手动输入时均以 deg 为单位.
 */
#ifndef STATE_FROM_MAVROS_H
#define STATE_FROM_MAVROS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <offb_msgs/AttitudeReference.h>
#include <offb_msgs/DroneState.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <bitset>

#include "math_utils.h"

class state_from_mavros {
 public:
  // constructed function
  state_from_mavros(void) : state_nh("~") {
    // [订阅] 无人机当前状态
    // 本话题来自飞控 (通过 mavros 功能包 plugins/sys_status.cpp 读取)
    state_sub = state_nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &state_from_mavros::state_cb, this);

    // [订阅] 无人机当前位置 坐标系: ENU 系
    // 此处注意: 所有状态量在飞控中均为 NED 系, 但在 ROS 中 mavros 将其转换为
    // ENU 系处理. 所以在 ROS 中, 所有和 mavros 交互的量都为 ENU 系.
    // 本话题来自飞控 (通过 mavros 功能包 plugins/local_position.cpp 读取),
    // 对应 mavlink 消息为 LOCAL_POSITION_NED (#32),
    // 对应的飞控中的 uORB 消息为 vehicle_local_position.msg.
    position_sub = state_nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, &state_from_mavros::pos_cb, this);

    // [订阅] 无人机当前速度 坐标系: ENU 系
    // 本话题来自飞控(通过 mavros 功能包 plugins/local_position.cpp 读取),
    // 对应 mavlink 消息为 LOCAL_POSITION_NED (#32),
    // 对应的飞控中的 uORB 消息为 vehicle_local_position.msg.
    velocity_sub = state_nh.subscribe<geometry_msgs::TwistStamped>(
        "/mavros/local_position/velocity_local", 10, &state_from_mavros::vel_cb,
        this);

    // [订阅] 无人机当前欧拉角 坐标系: ENU 系
    // 本话题来自飞控(通过 mavros 功能包 plugins/imu.cpp 读取),
    // 对应 mavlink 消息为 ATTITUDE (#30),
    // 对应的飞控中的 uORB 消息为 vehicle_attitude.msg.
    attitude_sub = state_nh.subscribe<sensor_msgs::Imu>(
        "/mavros/imu/data", 10, &state_from_mavros::att_cb, this);
  }

  // 变量声明
  offb_msgs::DroneState _DroneState;

 private:
  ros::NodeHandle state_nh;

  ros::Subscriber state_sub;
  ros::Subscriber position_sub;
  ros::Subscriber velocity_sub;
  ros::Subscriber attitude_sub;

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    _DroneState.connected = msg->connected;
    _DroneState.armed = msg->armed;
    _DroneState.mode = msg->mode;
  }

  void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    _DroneState.position[0] = msg->pose.position.x;
    _DroneState.position[1] = msg->pose.position.y;
    _DroneState.position[2] = msg->pose.position.z;
  }

  void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    _DroneState.velocity[0] = msg->twist.linear.x;
    _DroneState.velocity[1] = msg->twist.linear.y;
    _DroneState.velocity[2] = msg->twist.linear.z;
  }

  void att_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    Eigen::Quaterniond q_fcu =
        Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                           msg->orientation.y, msg->orientation.z);
    // Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

    _DroneState.attitude_q.w = q_fcu.w();
    _DroneState.attitude_q.x = q_fcu.x();
    _DroneState.attitude_q.y = q_fcu.y();
    _DroneState.attitude_q.z = q_fcu.z();

    _DroneState.attitude[0] = euler_fcu[0];
    _DroneState.attitude[1] = euler_fcu[1];
    _DroneState.attitude[2] = euler_fcu[2];

    _DroneState.attitude_rate[0] = msg->angular_velocity.x;
    _DroneState.attitude_rate[1] = msg->angular_velocity.x;
    _DroneState.attitude_rate[2] = msg->angular_velocity.x;
  }
};

#endif  // STATE_FROM_MAVROS_H
