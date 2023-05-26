#ifndef __DARKNET__HPP__

#define __DARKNET__HPP__

// Cluster Inference
#include "dummy_inference/dummy_inference.hpp"

// Darknet
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "darknet.h"

#include <sys/time.h>


class Darknet : public DummyInference
{
public:
  Darknet();
  ~Darknet();

  std::vector<ObjectDetection> get_detections(cv::Mat& image);

private:
  static char **demo_names;
  static image **demo_alphabet;
  static int demo_classes;

  static int nboxes = 0;
  static detection *dets = NULL;

  static network net;

  static cap_cv *cap;
  static float fps = 0;
  static float demo_thresh = 0;
  static int demo_ext_output = 0;
  static long long int frame_id = 0;
  static int demo_json_port = -1;
  static bool demo_skip_frame = false;


  static int avg_frames;
  static int demo_index = 0;
  static mat_cv** cv_images;

  static volatile int flag_exit;
  static int letter_box = 0;


};

#endif