#include "cluster/cluster.hpp"


ClusterManager::ClusterManager(int node_index, int number_of_nodes, int fps) : number_of_nodes_(number_of_nodes)
{
  for (int index = 0; index < number_of_nodes; index++)
  {
    this->cluster_nodes_.push_back({index, (index == node_index) ? true : false});
  }

  this->fps_ = static_cast<double>(fps);
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
  int selected_node_index = static_cast<int>((received_time / 1000000.0 - std::floor(received_time / 1000000.0)) * 1000000.0 * this->fps_) % this->number_of_nodes_;  // ms

  return this->cluster_nodes_[selected_node_index];
}