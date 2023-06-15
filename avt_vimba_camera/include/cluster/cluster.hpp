#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>


class ClusterManager
{
public:
  ClusterManager(int node_index, int number_of_nodes, double fps, double process_fps, double max_camera_cycle_time, double min_camera_cycle_time);
  ~ClusterManager();

  void register_base_timestamp(double timestamp);
  bool is_self_order(double timestamp);

private:
  bool is_in_range(double timestamp, double min_base_timestamp, double max_base_timestamp);

  double base_timestamp_;
  int pretimestamp_;

  double fps_;
  const int number_of_nodes_;
  const int node_index_;
  int frame_interval_;

  double max_camera_cycle_time_;
  double min_camera_cycle_time_;
};