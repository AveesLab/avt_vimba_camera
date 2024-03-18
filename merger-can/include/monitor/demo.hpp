#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <fstream>
#include <pthread.h>
#include <mutex>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// benchmark
#include <string>
#include <ctime>
#include <fstream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// Can
#include "can/canreceiver.hpp"
#include "monitor/types.h"


class MonitorDemo : public rclcpp::Node
{
public:
  explicit MonitorDemo();
  ~MonitorDemo();

  void can_receive();
  void can_show();

private:
  std::shared_ptr<CanReceiver> can_receiver_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void draw_image(cv_bridge::CvImagePtr cv_image, std::vector<ObjectDetection>& detections);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  static void* receive_thread(void* arg);
  static void* show_thread(void* arg);

  bool run_flag_;

  // Shared Resource
  std::queue<cv_bridge::CvImagePtr> image_queue_;
  std::vector<ObjectDetection> detections_;
  std::vector<std::vector<ObjectDetection>> detections_per_node_;

  std::vector<int> base_timestamp_;

  int number_of_nodes_ = 200;

  // pthread
  pthread_mutex_t mutex;
  pthread_mutex_t mutex_image;
  pthread_cond_t cond;
  pthread_t thread_receive;
  pthread_t thread_show;
};

