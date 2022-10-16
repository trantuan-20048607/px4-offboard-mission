#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt16.h>

#include "offboard_control.h"

double desire_z;                        // 巡航高度
auto num_waypoints = 0;                 // 航点个数
constexpr auto max_num_waypoints = 92;  // 最大航点个数
double delta_xyz;                       // 移动步长
double waypoint[96][3] = {0};           // 航点

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

void add_target(int i) {
  for (int j = 0; j < 3; ++j) {
    if (fabs(next_point[j] - waypoint[i + 1][j]) > 0.2) {
      if (next_point[j] > waypoint[i + 1][j])
        next_point[j] -= delta_xyz;
      else
        next_point[j] += delta_xyz;
    } else {
      next_point[j] = waypoint[i + 1][j];
    }
  }
}

bool arrived(int i) {
  return fabs(pos_drone[0] - waypoint[i + 1][0]) <= 0.1 &&
         fabs(pos_drone[1] - waypoint[i + 1][1]) <= 0.1 &&
         fabs(pos_drone[2] - waypoint[i + 1][2]) <= 0.1;
}

void fly_high(int i, double desire_z) {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);
  std::cout << "Climbing..." << std::endl;
  Eigen::Vector3d high_pos;

  high_pos[0] = pos_drone[0];
  high_pos[1] = pos_drone[1];
  high_pos[2] = 0.2;  // 最小高度, 缓慢提升

  auto show_warning = true;
  while (fabs(pos_drone[2] - desire_z) > 0.05) {
    if (state_drone.armed) {
      show_warning = true;
      offb.send_local_pos_setpoint(high_pos);
      if (high_pos[2] < desire_z) high_pos[2] += delta_xyz;
    } else if (show_warning) {
      std::cout << "Drone is disarmed." << std::endl;
      show_warning = false;
    }
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Reached desired altitude." << std::endl;
}

void fly_to(int i) {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);

  std::cout << "Flying to the next waypoint..." << std::endl;
  // 持续发送目标点
  while (!arrived(i)) {
    add_target(i);
    offb.send_local_pos_setpoint(next_point);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Reached above the next waypoint." << std::endl;
}

void land() {
  offboard_control::OffboardControl offb;
  ros::Rate rate(20.0);
  auto land_pos = end_point;

  std::cout << "Landing..." << std::endl;
  while (fabs(vel_drone[2]) > 0.1 || pos_drone[2] > 0.1) {
    offb.send_local_pos_setpoint(land_pos);
    if (land_pos[2] > -0.2)
      land_pos[2] -= delta_xyz;
    else
      land_pos[2] = -0.2;
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Landed and IDLE." << std::endl;
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
  land_pos[2] = -0.2;
  offboard_control::OffboardControl offb;
  ros::Time last_request = ros::Time::now();
  while (ros::Time::now() - last_request < ros::Duration(5.0)) {
    ros::spinOnce();
    rate.sleep();
    offb.send_pos_setpoint(land_pos, 0);
  }
  offb.send_serial_num(i);
  last_request = ros::Time::now();
  while (ros::Time::now() - last_request < ros::Duration(5.0)) {
    ros::spinOnce();
    rate.sleep();
    offb.send_pos_setpoint(land_pos, 0);
  }
  std::cout << "Item has been placed." << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "station_fly_control");
  ros::NodeHandle nh;

  auto arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  auto set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  auto position_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10, pos_cb);
  auto velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      "/mavros/local_position/velocity_local", 10, vel_cb);
  auto state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

  ros::Rate rate(20.0);
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  offb_set_mode.request.custom_mode = "AUTO.LAND";
  arm_cmd.request.value = false;

  offboard_control::OffboardControl offb;
  nh.getParam("station_fly_control/desire_z", desire_z);
  nh.getParam("station_fly_control/delta_xyz", delta_xyz);

  // 等待连接
  while (!state_drone.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "Current cruising altitude: " << desire_z << std::endl;

  // 输入航点
  do {
    std::cout << "Enter the num of waypoints: ";
    std::cin >> num_waypoints;
    std::cin.get();
  } while (num_waypoints <= 0 || num_waypoints >= max_num_waypoints);

  for (auto i = 1; i <= num_waypoints; ++i) {
    std::cout << "Enter the waypoint " << i << " (x, y, " << desire_z
              << "), separate by spaces: ";
    std::cin >> waypoint[i][0] >> waypoint[i][1];
    std::cin.get();
    waypoint[i][2] = desire_z;
  }
  waypoint[0][2] = desire_z;
  waypoint[num_waypoints + 1][2] = desire_z;

  // 等待进入 OFFBOARD 模式并解锁
  std::cout << "Waiting for setting to OFFBOARD mode and arming." << std::endl;
  while (state_drone.mode != "OFFBOARD" || !state_drone.armed) {
    ros::spinOnce();
    rate.sleep();
  }

  // 依次前往各个航点并执行对应任务
  for (auto i = 0; i < num_waypoints + 1; ++i) {
    start_point[0] = waypoint[i][0];
    start_point[1] = waypoint[i][1];
    start_point[2] = waypoint[i][2];
    end_point[0] = waypoint[i + 1][0];
    end_point[1] = waypoint[i + 1][1];
    end_point[2] = waypoint[i + 1][2];
    next_point[0] = start_point[0];
    next_point[1] = start_point[1];
    next_point[2] = desire_z;
    std::cout << "Departed from the waypoint " << i << "." << std::endl;
    fly_high(i, desire_z);
    fly_to(i);
    land();
    std::cout << "Arrived at the waypoint " << i + 1 << "." << std::endl;
    unload(i + 1);
    std::cout << "Object " << i + 1 << " is unloaded." << std::endl;
  }

  // 返回初始位置并降落
  std::cout
      << "All waypoints have been passed. Returning to the starting point."
      << std::endl;
  while (state_drone.mode != "AUTO.LAND") {
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Switched to AUTO.LAND mode." << std::endl;

  // 锁定无人机
  ros::Time land_time = ros::Time::now();
  while (state_drone.armed) {
    if (ros::Time::now() - land_time > ros::Duration(2.0)) {
      arming_client.call(arm_cmd);
    }
    ros::spinOnce();
    rate.sleep();
  }
  std::cout << "Drone is disarmed." << std::endl;
  return 0;
}
