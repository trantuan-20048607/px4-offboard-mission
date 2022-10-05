#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <iostream>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "station_mode_control");
  ros::NodeHandle nh("~");

  auto state_sub =
      nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  auto set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  auto arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  mavros_msgs::SetMode mode_cmd;

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Rate rate(10.0);

  auto mode_state = 0, input_num = 0;

  while (ros::ok()) {
    switch (mode_state) {
      // 用户输入
      case 0: {
        std::cout << "------------------------------------------------"
                  << std::endl;
        std::cout
            << "Set mode: 0 for arm, 1 for TAKEOFF, 2 for OFFBOARD, 3 for "
               "LAND, 4 for POSCTL, 5 for MISSION, 6 for LOITER, 7 for disarm."
            << std::endl;
        std::cin >> input_num;
        std::cin.get();
        mode_state = input_num + 1;
      } break;

      // arm
      case 1: {
        if (!current_state.armed) {
          arm_cmd.request.value = true;
          arming_client.call(arm_cmd);
          std::cout << "Arming..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Arm successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] Drone is armed.");
        }
      } break;

      // TAKEOFF
      case 2: {
        if (current_state.mode != "AUTO.TAKEOFF") {
          mode_cmd.request.custom_mode = "AUTO.TAKEOFF";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to TAKEOFF mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to AUTO.TAKEOFF mode successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] TAKEOFF mode set.");
        }
      } break;

      // OFFBOARD
      case 3: {
        if (current_state.mode != "OFFBOARD") {
          mode_cmd.request.custom_mode = "OFFBOARD";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to OFFBOARD mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to OFFBOARD mode successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] OFFBOARD mode set.");
        }
      } break;

      // LAND
      case 4: {
        if (current_state.mode != "AUTO.LAND") {
          mode_cmd.request.custom_mode = "AUTO.LAND";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to LAND mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to LAND mode successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] LAND mode set.");
        }
      } break;

      // POSCTL
      case 5: {
        if (current_state.mode != "POSCTL") {
          mode_cmd.request.custom_mode = "POSCTL";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to POSCTL mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to POSCTL mode successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] POSCTL mode set.");
        }
      } break;

      // MISSION
      case 6: {
        if (current_state.mode != "AUTO.MISSION") {
          mode_cmd.request.custom_mode = "AUTO.MISSION";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to AUTO.MISSION mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to AUTO.MISSION successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] AUTO.MISSION mode set.");
        }
      } break;

      case 7: {
        if (current_state.mode != "AUTO.LOITER") {
          mode_cmd.request.custom_mode = "AUTO.LOITER";
          set_mode_client.call(mode_cmd);
          std::cout << "Setting to AUTO.LOITER mode..." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Set to AUTO.LOITER mode successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] AUTO.LOITER mode set.");
        }
      } break;

      // disarm
      case 8: {
        if (current_state.armed) {
          arm_cmd.request.value = false;
          arming_client.call(arm_cmd);
          std::cout << "Disarming...." << std::endl;
        } else {
          mode_state = 0;
          std::cout << "Disarm successfully." << std::endl;
          ROS_INFO("[MODE CONTROL] Drone is disarmed.");
        }
      } break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
