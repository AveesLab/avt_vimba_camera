#pragma once

#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <string>

#include "darknet.h"
#include "inference/types.h"


class Darknet
{
public:
  Darknet(float thresh, char *cfg_file, char *weight_file);
  ~Darknet();

  std::vector<ObjectDetection> get_detections(cv::Mat& image);

  void preprocess(cv::Mat& input_image);
  void inference();
  std::vector<ObjectDetection> postprocess();

private:
  image get_image_from_mat(cv::Mat& image, int width, int height, int channel);
  std::vector<ObjectDetection> convert_detections();
  image mat_to_image(cv::Mat mat);

  char **demo_names;
  image **demo_alphabet;
  int demo_classes;

  image det_s_;
  int image_cols_;
  int image_rows_;

  int nboxes = 0;
  detection *dets = NULL;

  layer l;

  network net;

  cap_cv *cap;
  float fps = 0;
  float demo_thresh = 0;
  int demo_ext_output = 0;
  long long int frame_id = 0;
  int demo_json_port = -1;
  bool demo_skip_frame = false;


  int avg_frames;
  int demo_index = 0;
  mat_cv** cv_images;

  volatile int flag_exit;
  int letter_box = 0;
};