#ifndef __DUMMY__INFERENCE__HPP__

#define __DUMMY__INFERENCE__HPP__



class DummyInference
{
public:
  DummyInference() {}
  ~DummyInference() {}

  virtual std::vector<ObjectDetection> get_detections(cv::Mat& image);
};

#endif