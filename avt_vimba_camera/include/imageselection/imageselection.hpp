#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>


class ImageSelection
{
public:
  ImageSelection(int node_index, int number_of_nodes, double local_fps, double margin);
  ~ImageSelection();

  void RegisterBaseTimestamp(double init_timestamp);
  bool IsSelfOrder(double timestamp);
  double GetEstimatedTimestamp();

private:
  bool IsInRange(double timestamp, double estimated_timestamp);

  bool is_compute_node;
  double estimated_timestamp_;
  double local_fps_;
  double inverse_of_fps_;
  const int number_of_nodes_;
  const int node_index_;
  const double margin_;
  double max_estimated_timestamp;
  double min_estimated_timestamp;
};