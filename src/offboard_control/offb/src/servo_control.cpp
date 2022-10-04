#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include "offboard_control.h"

int main(int argc, char** argv) {
  uint8_t num;
  ros::init(argc, argv, "servo_control");
  ros::NodeHandle nh;
  offboard_control::OffboardControl offb;
  while (ros::ok()) {
    switch (num) {
      case 0:
        std::cout << "------------------------------------------------"
                  << std::endl;
        std::cout << "Input: 1 for servo_1, 2 for servo_2, 3 for servo_3, 6 "
                     "for all servos."
                  << std::endl;
        std::cin >> num;
        std::cin.get();
      case 1:
        offb.send_serial_num(num);
        std::cout << "成功发送指令" << std::endl;
        num = 0;
        break;
      case 2:
        offb.send_serial_num(num);
        std::cout << "成功发送指令" << std::endl;
        num = 0;
        break;
      case 3:
        offb.send_serial_num(num);
        std::cout << "成功发送指令" << std::endl;
        num = 0;
        break;
      case 6:
        offb.send_serial_num(num);
        std::cout << "成功发送指令" << std::endl;
        num = 0;
        break;
      default:
        num = 0;
        break;
    }
  }
  return 0;
}
