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

  void register_base_timestamp(int index, int timestamp);
  bool is_self_order(double timestamp);

private:
  bool is_in_range(int remaked_timestamp, int & reserved_timestamp);
  bool is_set_timestamp(int remaked_timestamp);

  std::vector<int> cluster_nodes_;

  double fps_;
  const int number_of_nodes_;
  int frame_interval_;
  int pretimestamp_;

  double max_camera_cycle_time_;
  double min_camera_cycle_time_;
};