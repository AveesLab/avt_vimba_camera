#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>


struct ClusterNode
{
  int id = -1;
  bool self = false;
};

class ClusterManager
{
public:
  ClusterManager(int node_index, int number_of_nodes, int fps);
  ~ClusterManager();

  bool is_self_order(double timestamp);

private:
  ClusterNode& select_node(double received_time);

  std::vector<ClusterNode> cluster_nodes_;

  double fps_;
  const int number_of_nodes_;
};