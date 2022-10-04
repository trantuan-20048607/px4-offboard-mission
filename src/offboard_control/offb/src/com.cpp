#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt16.h>

#include "offboard_control.h"

double desire_z;         // 无人机将一直以这个高度飞行
double desire_vz = 0.5;  // 无人机以这个速度起飞降落
int num_fly;             // 无人机的航点个数
double change;
double fly_point[100][3];  // 存储无人机的航点
Eigen::Vector3d start_point, next_point, end_point;
Eigen::Vector3d start_vel, landing_vel, slow_vel;
Eigen::Vector3d pos_drone, vel_drone;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  pos_drone[0] = msg->pose.position.x;
  pos_drone[1] = msg->pose.position.y;
  pos_drone[2] = msg->pose.position.z;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  vel_drone[0] = msg->twist.linear.x;
  vel_drone[1] = msg->twist.linear.y;
  vel_drone[2] = msg->twist.linear.z;
}

mavros_msgs::State state_drone;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
  state_drone.connected = msg->connected;
  state_drone.armed = msg->armed;
  state_drone.mode = msg->mode;
}

// 改变下一个时刻向飞控发送的位置信息
void add_target(int i) {
  for (int j = 0; j < 3; j++) {
    if (abs(next_point[j] - fly_point[i + 1][j]) > 0.3) {
      if (next_point[j] > fly_point[i + 1][j]) {
        next_point[j] -= change;
      } else {
        next_point[j] += change;
      }
    } else {
      next_point[j] = fly_point[i + 1][j];
    }
  }
}

// 判断无人机是否到达目标点
int not_arrived(int i) {
  double x_mis = abs(pos_drone[0] - fly_point[i + 1][0]),
         y_mis = abs(pos_drone[1] - fly_point[i + 1][1]),
         z_mis = abs(pos_drone[2] - fly_point[i + 1][2]);
  return (x_mis <= 0.15 && y_mis <= 0.15 && z_mis <= 0.15 ? 0 : 1);
}

void fly_high(int i, double desire_z) {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);
  std::cout << "正在上升中" << std::endl;
  Eigen::Vector3d high_pos;
  high_pos[0] = pos_drone[0];
  high_pos[1] = pos_drone[1];
  high_pos[2] = 0.3;
  while (abs(pos_drone[2] - desire_z) > 0.05) {
    if (state_drone.armed) {
      offb.send_local_pos_setpoint(high_pos);
      if (high_pos[2] < desire_z + 0.1) {
        high_pos[2] += change;
      }
    } else {
      std::cout << "无人机已上锁" << std::endl;
    }
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "已经到达期望高度" << std::endl;
}

void fly_to(int i) {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);
  std::cout << "正在飞往下一个目标点" << std::endl;
  while (not_arrived(i)) {
    add_target(i);
    offb.send_local_pos_setpoint(next_point);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "已经到达目标点上方" << std::endl;
}

void landing() {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);
  Eigen::Vector3d land_pos = end_point;
  std::cout << "准备降落" << std::endl;
  while (abs(vel_drone[2]) > 0.1 || pos_drone[2] > 0.1) {
    offb.send_local_pos_setpoint(land_pos);
    if (land_pos[2] > -0.5) {
      land_pos[2] -= change;
    } else {
      land_pos[2] = -0.5;
    }
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "降落成功, 怠速旋转" << std::endl;
}

void unload(int i) {
  Eigen::Vector3d land_pos;
  ros::Rate rate(20.0);
  while (pos_drone[2] > 0.2) {
    ros::spinOnce();
    rate.sleep();
  }
  land_pos[0] = pos_drone[0];
  land_pos[1] = pos_drone[1];
  land_pos[2] = -0.5;
  offboard_control::OffboardControl offb;
  ros::Time last_request = ros::Time::now();
  while (ros::Time::now() - last_request < ros::Duration(3.0)) {
    ros::spinOnce();
    rate.sleep();
    offb.send_pos_setpoint(land_pos, 0);
  }
  offb.send_serial_num(i);
  last_request = ros::Time::now();
  while (ros::Time::now() - last_request < ros::Duration(2.0)) {
    ros::spinOnce();
    rate.sleep();
    offb.send_pos_setpoint(land_pos, 0);
  }
  std::cout << "投放完毕" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "com");
  ros::NodeHandle nh;
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>  // 定义服务, 用来解锁无人机
      ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>  // 定义服务, 用来改变无人机的模式
      ("mavros/set_mode");
  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10, pos_cb);
  ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      "/mavros/local_position/velocity_local", 10, vel_cb);
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::Rate rate(20.0);
  mavros_msgs::SetMode offb_set_mode;  // 定义改变模式的变量
  mavros_msgs::CommandBool arm_cmd;    // 定义解锁飞机的变量
  offb_set_mode.request.custom_mode = "AUTO.LAND";  // 更改飞行模式为手动
  arm_cmd.request.value = false;                    // 对飞机上锁
  offboard_control::OffboardControl offb;
  nh.getParam("com_node/desire_z", desire_z);
  nh.getParam("com_node/change", change);
  // ros::param::get("~desire_z", desire_z);
  // ros::param::get("~change", change);
  while (!state_drone.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "无人机的巡航高度是: " << desire_z << std::endl;
  std::cout << "请输入航点个数: " << std::endl;
  std::cin >> num_fly;
  std::cin.get();
  for (int i = 1; i <= num_fly; i++) {
    std::cout << "请输入第" << i << "个航点: ";
    std::cin >> fly_point[i][0] >> fly_point[i][1] >> fly_point[i][2];
    fly_point[i][2] = desire_z;
  }
  fly_point[0][2] = desire_z;
  fly_point[num_fly + 1][2] = desire_z;
  std::cout << "等待OFFBOARD模式以及解锁飞机" << std::endl;
  while (state_drone.mode != "OFFBOARD" || !state_drone.armed) {
    ros::spinOnce();
    rate.sleep();
  }
  for (int i = 0; i < num_fly + 1; i++) {
    start_point[0] = fly_point[i][0];
    start_point[1] = fly_point[i][1];
    start_point[2] = fly_point[i][2];
    end_point[0] = fly_point[i + 1][0];
    end_point[1] = fly_point[i + 1][1];
    end_point[2] = fly_point[i + 1][2];
    next_point[0] = start_point[0];
    next_point[1] = start_point[1];
    next_point[2] = desire_z;
    fly_high(i, desire_z);
    fly_to(i);
    landing();
    unload(i + 1);
    std::cout << "第" << i + 1 << "个点的任务已经完成" << std::endl;
  }
  std::cout << "所有任务点飞行结束" << std::endl;
  while (state_drone.mode != "AUTO.LAND") {
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "已经成功切换到AUTO.LAND模式" << std::endl;
  ros::Time land_time = ros::Time::now();
  while (state_drone.armed) {
    if (ros::Time::now() - land_time > ros::Duration(2.0)) {
      arming_client.call(arm_cmd);
    }
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "无人机成功上锁" << std::endl;
  return 0;
}
