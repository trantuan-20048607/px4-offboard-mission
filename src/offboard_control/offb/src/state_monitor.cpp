#include "state_from_mavros.h"

int main(int argc, char **argv) {
  constexpr int obj_width = 16;
  int header = 1;
  ros::init(argc, argv, "state_monitor");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  state_from_mavros::StateFromMavros state;
  while (ros::ok()) {
    std::cout << "------------------------------------------------"
              << std::endl;
    std::cout << std::setw(obj_width) << "header: " << std::setw(obj_width)
              << header << std::endl;
    std::cout << std::setw(obj_width) << "connected: " << std::setw(obj_width)
              << (state.drone_state_.connected ? "true" : "false") << std::endl;
    std::cout << std::setw(obj_width) << "armed: " << std::setw(obj_width)
              << (state.drone_state_.armed ? "true" : "false") << std::endl;
    std::cout << std::setw(obj_width) << "mode: " << std::setw(obj_width)
              << state.drone_state_.mode << std::endl;
    std::cout << std::setw(obj_width) << "position: ";
    std::cout << std::setw(obj_width) << state.drone_state_.position[0]
              << std::setw(obj_width) << state.drone_state_.position[1]
              << std::setw(obj_width) << state.drone_state_.position[2]
              << std::endl;
    std::cout << std::setw(obj_width) << "velocity: ";
    std::cout << std::setw(obj_width) << state.drone_state_.velocity[0]
              << std::setw(obj_width) << state.drone_state_.velocity[1]
              << std::setw(obj_width) << state.drone_state_.velocity[2]
              << std::endl;
    std::cout << std::setw(obj_width) << "attitude q: ";
    std::cout << std::setw(obj_width) << state.drone_state_.attitude_q.x
              << std::setw(obj_width) << state.drone_state_.attitude_q.y
              << std::setw(obj_width) << state.drone_state_.attitude_q.z
              << std::setw(obj_width) << state.drone_state_.attitude_q.w
              << std::endl;
    Eigen::Quaterniond attitude_q{
        state.drone_state_.attitude_q.w, state.drone_state_.attitude_q.x,
        state.drone_state_.attitude_q.y, state.drone_state_.attitude_q.z};
    auto euler_angle = math_utils::quaternion_to_euler(attitude_q);
    std::cout << std::setw(obj_width) << "attitude e: ";
    std::cout << std::setw(obj_width) << euler_angle.x() << std::setw(obj_width)
              << euler_angle.y() << std::setw(obj_width) << euler_angle.z()
              << std::endl;
    std::cout << std::endl;
    ros::spinOnce();
    rate.sleep();
    header++;
  }
}
