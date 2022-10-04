#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "csi_camera.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision");
  ros::NodeHandle nh;
  ros::Rate rate(10.0);
  int capture_width = 1280;
  int capture_height = 720;
  int display_width = 1280;
  int display_height = 720;
  int framerate = 30;
  int flip_method = 0;
  auto pipeline = csi_camera::gstreamer_pipeline(capture_width, capture_height,
                                                 display_width, display_height,
                                                 framerate, flip_method);
  auto cap = csi_camera::open_camera(pipeline);
  if (!cap.isOpened()) {
    std::cout << "Failed to open camera. Disable all vision functions." << std::endl;
  } else {
    // TODO 此处填写识别代码
  }
  cap.release();
  return 0;
}
