#ifndef CSI_CAMERA_H
#define CSI_CAMERA_H

#include <string>

inline std::string gstreamer_pipeline(int capture_width, int capture_height,
                                      int display_width, int display_height,
                                      int framerate, int flip_method) {
  return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" +
         std::to_string(capture_width) + ", height=(int)" +
         std::to_string(capture_height) + ", framerate=(fraction)" +
         std::to_string(framerate) +
         "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) +
         " ! video/x-raw, width=(int)" + std::to_string(display_width) +
         ", height=(int)" + std::to_string(display_height) +
         ", format=(string)BGRx ! videoconvert ! video/x-raw, "
         "format=(string)BGR ! appsink";
}

class CSICamera {
 public:
  CSICamera() {
  }
 private:
};

#endif  // CSI_CAMERA_H
