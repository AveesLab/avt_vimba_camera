#ifndef __DUMMY__INFERENCE__YOLOV7__HPP__

#define __DUMMY__INFERENCE__YOLOV7__HPP__

#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/opencv.hpp>

// TensorRT
#include "dummy_inference/include/yolov7/include/config.h"
#include "dummy_inference/include/yolov7/include/model.h"
#include "dummy_inference/include/yolov7/include/cuda_utils.h"
#include "dummy_inference/include/yolov7/include/logging.h"
#include "dummy_inference/include/yolov7/include/utils.h"
#include "dummy_inference/include/yolov7/include/preprocess.h"
#include "dummy_inference/include/yolov7/include/postprocess.h"

// OpenCV
#include <opencv2/opencv.hpp>


struct ObjectDetection
{
  int id;
  int center_x;
  int center_y;
  int width_half;
  int height_half;
};

using namespace nvinfer1;

class Yolov7
{
public:
  Yolov7(std::string model_path);

  ~Yolov7();

  std::vector<ObjectDetection> get_detections(cv::Mat& image);

private:
  std::vector<std::vector<Detection>> inference(std::vector<cv::Mat>& image_batch);
  std::vector<ObjectDetection> convert_detections(cv::Mat& image, std::vector<std::vector<Detection>>& result_batch);

  // initialize Yolov7 network model
  void initialize_model(std::string model_path);

  std::string model_path_;

  // TensorRT-Yolov7 members
  IRuntime* runtime_ = nullptr;
  ICudaEngine* engine_ = nullptr;
  IExecutionContext* context_ = nullptr;
  cudaStream_t stream_;
  float* device_buffers_[2];
  float* output_buffer_host_ = nullptr;
};

#endif