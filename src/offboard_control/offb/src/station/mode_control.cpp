#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <iostream>

// 无人机当前状态 [包含上锁状态], 从飞控中读取
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "station_mode_control");
  ros::NodeHandle nh("~");

  // [订阅] 无人机当前状态
  // 本话题来自飞控, 通过 plugins/sys_status.cpp
  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

  // [服务] 修改系统模式
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  mavros_msgs::SetMode mode_cmd;

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Rate rate(10.0);

  int mode_state = 0;
  int flag;

  while (ros::ok()) {
    switch (mode_state) {
      // Input
      case 0:
        std::cout << "------------------------------------------------"
                  << std::endl;
        std::cout
            << "Set mode: 0 for arm, 1 for TAKEOFF, 2 for OFFBOARD, 3 for "
               "LAND, 4 for POSCTL, 5 for MISSION, 6 for LOITER, 7 for disarm."
            << std::endl;
        std::cin >> flag;
        std::cin.get();

        if (flag == 0) {
          mode_state = 1;
          break;
        } else if (flag == 1) {
          mode_state = 2;
          break;
        } else if (flag == 2) {
          mode_state = 3;
        } else if (flag == 3) {
          mode_state = 4;
        } else if (flag == 4) {
          mode_state = 5;
        } else if (flag == 5) {
          mode_state = 6;
        } else if (flag == 6) {
          mode_state = 7;
        } else if (flag == 7) {
          mode_state = 8;
        }
        break;

      // Arm
      case 1:
        if (!current_state.armed) {
          arm_cmd.request.value = true;
          arming_client.call(arm_cmd);
          std::cout << "Arming..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Arm succeeded." << std::endl;
        }
        break;

      // TAKEOFF
      case 2:
        if (current_state.mode != "AUTO.TAKEOFF") {
          mode_cmd.request.custom_mode = "AUTO.TAKEOFF";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to TAKEOFF mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to AUTO.TAKEOFF mode succeeded." << std::endl;
        }
        break;

      // OFFBOARD
      case 3:
        if (current_state.mode != "OFFBOARD") {
          mode_cmd.request.custom_mode = "OFFBOARD";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to OFFBOARD mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to OFFBOARD mode succeeded." << std::endl;
        }
        break;

      // LAND
      case 4:
        if (current_state.mode != "AUTO.LAND") {
          mode_cmd.request.custom_mode = "AUTO.LAND";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to LAND mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to LAND mode succeeded." << std::endl;
        }
        break;

      // POSCTL
      case 5:
        if (current_state.mode != "POSCTL") {
          mode_cmd.request.custom_mode = "POSCTL";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to POSCTL mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to POSCTL mode succeeded." << std::endl;
        }
        break;

      // MISSION
      case 6:
        if (current_state.mode != "AUTO.MISSION") {
          mode_cmd.request.custom_mode = "AUTO.MISSION";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to MISSION mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to RTL MISSION succeeded." << std::endl;
        }
        break;

      case 7:
        if (current_state.mode != "AUTO.LOITER") {
          mode_cmd.request.custom_mode = "AUTO.LOITER";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to AUTO.LOITER mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to AUTO.LOITER mode succeeded." << std::endl;
        }
        break;

      case 8:
        if (current_state.armed) {
          arm_cmd.request.value = false;
          arming_client.call(arm_cmd);
          std::cout << "Disarming...." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Disarm succeeded." << std::endl;
        }
        break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
