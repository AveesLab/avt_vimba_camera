#include "cluster/cluster.hpp"


ClusterManager::ClusterManager(int node_index, int number_of_nodes, double fps, double process_fps) : number_of_nodes_(number_of_nodes), fps_(fps)
{
  double gap = fps_ / process_fps;
  int start_index = static_cast<int>(gap / static_cast<double>(number_of_nodes) * static_cast<double>(node_index));

  for (int index = 0; index < this->fps_; index++)
  {
    this->cluster_nodes_.push_back({index % number_of_nodes, (index % static_cast<int>(gap) == start_index) ? true : false});
  }
}

ClusterManager::~ClusterManager()
{
  this->cluster_nodes_.clear();
}

bool ClusterManager::is_self_order(double timestamp)
{
  ClusterNode selected_node = this->select_node(timestamp);

  return selected_node.self;
}

ClusterNode& ClusterManager::select_node(double received_time)
{
  // Algorithm
  int selected_node_index = static_cast<int>((received_time - std::floor(received_time)) * (1000.0 * this->fps_)) / 1000;  // ms

  return this->cluster_nodes_[selected_node_index];
}