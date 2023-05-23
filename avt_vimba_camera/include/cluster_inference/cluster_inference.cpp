#include "cluster_inference/cluster_inference.hpp"


ClusterManager::ClusterManager(int node_index, int number_of_nodes, int interval) : number_of_nodes_(number_of_nodes), interval_(interval)
{
  for (int index = 0; index < this->number_of_nodes_; index++)
  {
    this->cluster_nodes_.push_back({static_cast<short int>(index), (index == node_index) ? true : false});
  }
}

ClusterManager::~ClusterManager()
{
  this->cluster_nodes_.clear();
}

bool ClusterManager::is_self_order(double timestamp)
{
  double received_time = timestamp * 1000.0;

  ClusterNode selected_node = this->select_node(received_time);

  return selected_node.self;
}

ClusterNode& ClusterManager::select_node(double received_time)
{
  // Algorithm
  int selected_node_index = static_cast<int>(received_time) / this->interval_ % this->number_of_nodes_;  // ms

  return this->cluster_nodes_[selected_node_index];
}