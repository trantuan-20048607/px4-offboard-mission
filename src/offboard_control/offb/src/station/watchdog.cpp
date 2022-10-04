#include "offboard_control.h"

Eigen::Vector3d vel_pos{0, 0, 0};
Eigen::Vector3d pos_drone;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  pos_drone = {msg->pose.position.x, msg->pose.position.y,
               msg->pose.position.z};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "station_watchdog");
  ros::NodeHandle nh;
  ros::Rate rate(20.0);
  auto state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  auto position_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10, pos_cb);
  offboard_control::OffboardControl offb;
  while (ros::ok() && !current_state.connected) {
    ROS_WARN_ONCE("[WATCHDOG] Waiting for connection...");
    offb.send_velxyz_setpoint(vel_pos, 0);
    ros::spinOnce();
    rate.sleep();
  }
  auto last_time = ros::Time::now();
  bool show_warning = true;
  while (ros::ok()) {
    if (current_state.armed && current_state.mode == "OFFBOARD") {
      show_warning = true;
    } else {
      if (!current_state.armed && show_warning) {
        std::stringstream warn_ss;
        warn_ss << "[WATCHDOG] Drone is armed but not in OFFBOARD mode. Send default "
            "velocity point. Current mode is " << current_state.mode << ".";
        ROS_WARN(warn_ss.str().c_str());
        show_warning = false;
      }
      offb.send_velxyz_setpoint(vel_pos, 0);
    }
    if ((ros::Time::now() - last_time) > ros::Duration(3.0)) {
      ROS_INFO("[WATCHDOG] The watchdog process is running normally.");
      last_time = ros::Time::now();
    }
    ros::spinOnce();
    rate.sleep();
  }
}
