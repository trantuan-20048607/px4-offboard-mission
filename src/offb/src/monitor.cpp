#include "state_from_mavros.h"

using namespace std;

int main(int argc, char **argv) {
  int header = 1;
  ros::init(argc, argv, "monitor_node");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  state_from_mavros state_from_mavros_;
  while (ros::ok()) {
    cout << "--------------------------------" << endl;
    cout << setw(12) << "header: " << setw(12) << header << endl;
    cout << setw(12) << "connected: " << setw(12)
         << (state_from_mavros_._DroneState.connected ? "true" : "false")
         << endl;
    cout << setw(12) << "armed: " << setw(12)
         << (state_from_mavros_._DroneState.armed ? "true" : "false") << endl;
    cout << setw(12) << "mode: " << setw(12) << state_from_mavros_._DroneState.mode << endl;
    cout << setw(12) << "position: ";
    cout << setw(12) << state_from_mavros_._DroneState.position[0] << setw(12)
         << state_from_mavros_._DroneState.position[1] << setw(12)
         << state_from_mavros_._DroneState.position[2] << endl;
    cout << setw(12) << "velocity: ";
    cout << setw(12) << state_from_mavros_._DroneState.velocity[0] << setw(12)
         << state_from_mavros_._DroneState.velocity[1] << setw(12)
         << state_from_mavros_._DroneState.velocity[2] << endl;
    cout << setw(12) << "attitude q: ";
    cout << setw(12) << state_from_mavros_._DroneState.attitude_q.x << setw(12)
         << state_from_mavros_._DroneState.attitude_q.y << setw(12)
         << state_from_mavros_._DroneState.attitude_q.z << setw(12)
         << state_from_mavros_._DroneState.attitude_q.w << endl;
    Eigen::Quaterniond attitude_q{state_from_mavros_._DroneState.attitude_q.w,
                                  state_from_mavros_._DroneState.attitude_q.x,
                                  state_from_mavros_._DroneState.attitude_q.y,
                                  state_from_mavros_._DroneState.attitude_q.z};
    auto euler_angle = quaternion_to_euler(attitude_q);
    cout << setw(12) << "attitude e: ";
    cout << setw(12) << euler_angle.x() << setw(12) << euler_angle.y()
         << setw(12) << euler_angle.z() << endl;
    ros::spinOnce();
    rate.sleep();
    header++;
  }
}
