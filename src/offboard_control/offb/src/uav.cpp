#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>

#include <iostream>

// 无符号单字节整型数据, 这里代表的是投放的顺序, 初始化为 5.
// 1, 2, 3 代表 1 号, 2 号, 3号投放. 4代表打开所有舵机. 5 没有意义.
uint8_t throw_n = 5;
// 6 代表还原所有舵机
uint8_t back = 6;

// 这里 serial_port::serial_message 是自己定义的 msg, 代表投放的顺序
void SerialCallback(const std_msgs::UInt8& msg) {
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
  ros::init(argc, argv, "uav");
  // 创建句柄 (虽然后面没用到这个句柄, 但如果不创建, 运行时进程会出错)
  ros::NodeHandle n;
  // 创建一个 Serial 类, 实例化为 sp
  serial::Serial sp;
  // 创建一个订阅者, 订阅的话题类型为 /landing_quadrotor_node/throw_message ,
  // 数据缓存区为 10 个字节, 回调函数为 SerialCallback
  ros::Subscriber sub = n.subscribe("/servo", 1, &SerialCallback);

  // 创建 timeout
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  // 设置要打开的串口名称
  std::string throw_serial_port;
  int Baudrate;
  // 将串口名称定义为 throw_serial_port, 可以通过 launch 文件进行更改, 默认为
  // /dev/ttyUSB0
  n.param<std::string>("uav_node/throw_serial_port", throw_serial_port,
                       "/dev/ttyUSB0");

  // 将波特率参数定义为 Baudrate, 可以通过 launch 文件进行更改. 默认为 9600.
  n.param<int>("uav_node/Baudrate", Baudrate, 9600);

  sp.setPort(throw_serial_port);
  // 设置串口通信的波特率
  sp.setBaudrate(Baudrate);
  // 串口设置 timeout
  sp.setTimeout(to);

  try {
    // 打开串口
    sp.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }

  // 判断串口是否打开成功
  if (sp.isOpen()) {
    // 这里将串口名称和波特率打印出来,
    // 在 ROS 中将 launch 文件可以修改的参数引出是很好的习惯,
    // 因为可以看到是否更改成功, 在 debug 时有很大的作用
    std::cout << "throw_serial_port:" << throw_serial_port << std::endl;
    std::cout << "Baudrate:" << Baudrate << std::endl;
  } else {
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
