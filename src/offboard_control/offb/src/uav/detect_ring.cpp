#include <offb_msgs/DetectRing.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "csi_camera.h"

constexpr int capture_width = 1280;
constexpr int capture_height = 720;
constexpr int display_width = 1280;
constexpr int display_height = 720;
constexpr int framerate = 30;
constexpr int flip_method = 0;

cv::Point find_ring_center(const cv::Mat& img) {
  // 显示原图
  // cv::imshow("img", img);

  // 模糊算法预处理, 去除独立噪点
  auto filtered = cv::Mat();
  cv::bilateralFilter(img, filtered, 3, 1024, 1024);
  // cv::imshow("filtered", filtered);

  // Canny 边缘检测
  auto edge_detected = cv::Mat();
  cv::Canny(filtered, edge_detected, 24, 144);
  // cv::imshow("edge_detected", edge_detected);

  // 闭-开运算, 融合边缘检测产生的噪声
  auto closed = cv::Mat();
  cv::morphologyEx(edge_detected, closed, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
  auto close_opened = cv::Mat();
  cv::morphologyEx(closed, close_opened, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  // cv::imshow("close_opened", close_opened);

  // 提取并画出圆形边缘
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(close_opened, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  cv::Mat contour_image(close_opened.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Point min_radius_center;
  double min_radius = 8192;
  for (size_t idx = 0; idx < contours.size(); idx++) {
    // 对于圆形, 面积 S = pi * r^2, 周长 C = 2 * pi * r.
    // 仅通过这两个值算出圆周率的值 pi = C^2 / (4 * S), 并与真实值作比较,
    // 即可筛选出圆形边缘.
    auto area = cv::contourArea(contours[idx]);
    auto circumference = cv::arcLength(contours[idx], true);
    // 圆度 = C^2 / (4 * S) / pi
    auto roundness = circumference * circumference / (4 * area) / M_PI;
    // 近似半径 r = 2 * S / C
    auto radius = 2 * area / circumference;
    if (radius > 32 && exp(roundness) <= M_PI) {
      cv::drawContours(contour_image, contours, idx, cv::Scalar(255, 255, 255));
      if (radius < min_radius) {
        cv::Moments M = cv::moments(contours[idx]);
        min_radius_center = {M.m10 / M.m00, M.m01 / M.m00};
        min_radius = radius;
      }
    }
  }
  // 作近似半径最小的圆的重心
  if (min_radius < 8192)
    cv::circle(contour_image, min_radius_center, 2, cv::Scalar(255, 255, 255),
               2);
  // cv::imshow("contour_image", contour_image);

  return min_radius_center;
}

cv::VideoCapture cap;

bool detect_ring_service_cb(offb_msgs::DetectRing::Request& req,
                            offb_msgs::DetectRing::Response& res) {
  auto image = cv::Mat();
  auto ret = cap.read(image);
  if (ret) {
    auto ring_center = find_ring_center(image);
    // TODO 填写坐标系变换内容
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_detect_ring");
  ros::NodeHandle nh;

  auto pipeline = csi_camera::gstreamer_pipeline(capture_width, capture_height,
                                                 display_width, display_height,
                                                 framerate, flip_method);
  cap = csi_camera::open_camera(pipeline);
  if (!cap.isOpened()) {
    std::cout << "Failed to open camera. Disable all vision functions."
              << std::endl;
    ROS_WARN("Failed to open camera. Disable all vision functions.");
    return 0;
  }

  auto detect_ring_service =
      nh.advertiseService("detect_ring", detect_ring_service_cb);
  ros::spin();

  cap.release();
  return 0;
}
