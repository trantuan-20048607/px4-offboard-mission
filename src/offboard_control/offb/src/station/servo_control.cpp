#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include "offboard_control.h"

int main(int argc, char** argv) {
  uint8_t num = 0;
  ros::init(argc, argv, "station_servo_control");
  ros::NodeHandle nh;
  offboard_control::OffboardControl offb;
  while (ros::ok()) {
    switch (num) {
      // 用户输入
      case 0: {
        std::cout << "------------------------------------------------"
                  << std::endl;
        std::cout << "Control servos: 1 for servo_1, 2 for servo_2, 3 for "
                     "servo_3, 6 for all servos."
                  << std::endl;
        std::cin >> num;
        std::cin.get();
      } break;

      // 发送指令
      case 1:
      case 2:
      case 3:
      case 6: {
        offb.send_serial_num(num);
        std::cout << "Servo serial command sent successfully." << std::endl;
        std::stringstream info_ss;
        info_ss << "[SERVO CONTROL] Send servo serial command " << num
                << " successfully.";
        ROS_INFO(info_ss.str().c_str());
        num = 0;
      } break;

      // 无效状态
      default: {
        std::cout << "Invalid command." << std::endl;
        num = 0;
      } break;
    }
  }
  return 0;
}
