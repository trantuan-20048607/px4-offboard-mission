#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>

#include <iostream>

// 无符号单字节整型数据, 这里代表的是投放的顺序, 初始化为 5.
// 1, 2, 3 代表 1 号, 2 号, 3号投放. 4 代表打开所有舵机. 5 没有意义.
uint8_t throw_n = 5;
// 6 代表还原所有舵机
uint8_t back = 6;

void serial_cb(const std_msgs::UInt8& msg) {
  throw_n = msg.data;
  if (throw_n > 48) {
    throw_n -= 48;
  }
  if (throw_n > 3) {
    throw_n = 6;
  }
  std::cout << static_cast<int>(throw_n) << "号投放" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_servo");
  ros::NodeHandle nh;
  serial::Serial sp;
  ros::Subscriber sub = nh.subscribe("/servo", 1, &serial_cb);

  serial::Timeout to = serial::Timeout::simpleTimeout(100);

  std::string throw_serial_port;
  int baud_rate;
  nh.param<std::string>("uav_servo_node/throw_serial_port", throw_serial_port,
                        "/dev/ttyUSB0");
  nh.param<int>("uav_servo_node/baud_rate", baud_rate, 9600);

  sp.setPort(throw_serial_port);
  sp.setBaudrate(baud_rate);
  sp.setTimeout(to);

  try {
    sp.open();
  } catch (...) {
    ROS_ERROR_STREAM("Unable to open port " + throw_serial_port + ".");
    return -1;
  }

  // 10 Hz
  ros::Rate loop_rate(10);
  int flag = 0;
  ros::Time last = ros::Time::now();
  int i = 0;
  while (ros::ok()) {
    switch (throw_n) {
      case 1:
        sp.write(&throw_n, 1);
        throw_n = 5;
        break;
      case 2:
        sp.write(&throw_n, 1);
        throw_n = 5;
        break;
      case 3:
        sp.write(&throw_n, 1);
        throw_n = 5;
        break;
      case 6:
        sp.write(&throw_n, 1);
        throw_n = 5;
        break;
      default:
        throw_n = 5;
        break;
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  sp.write(&back, 1);
  sp.close();

  return 0;
}
