#include "cluster/cluster.hpp"
#include <iostream>


ClusterManager::ClusterManager(int node_index, int number_of_nodes, double fps, double process_fps, double max_camera_cycle_time, double min_camera_cycle_time) : number_of_nodes_(number_of_nodes), fps_(fps), node_index_(node_index)
{
  this->base_node_index_ = this->number_of_nodes_;

  this->min_pretimestamp_ = -1;
  this->max_pretimestamp_ = -1;

  this->frame_interval_ = fps / process_fps / this->number_of_nodes_;

  this->max_camera_cycle_time_ = max_camera_cycle_time;
  this->min_camera_cycle_time_ = min_camera_cycle_time;
}

ClusterManager::~ClusterManager()
{
}

void ClusterManager::register_base_timestamp(double timestamp, int node_index)
{
  if (this->base_node_index_ > node_index)
  {
    this->base_node_index_ = node_index;

    this->min_pretimestamp_ = timestamp * 1000.0 + this->min_camera_cycle_time_ * this->frame_interval_ * (((this->node_index_ - this->base_node_index_) >= 0) ? (this->node_index_ - this->base_node_index_) : this->number_of_nodes_ + (this->node_index_ - this->base_node_index_));
    this->max_pretimestamp_ = timestamp * 1000.0 + this->max_camera_cycle_time_ * this->frame_interval_ * (((this->node_index_ - this->base_node_index_) >= 0) ? (this->node_index_ - this->base_node_index_) : this->number_of_nodes_ + (this->node_index_ - this->base_node_index_));
  }
}

bool ClusterManager::is_self_order(double timestamp)
{
  if (this->base_node_index_ != this->number_of_nodes_)
  {
    timestamp *= 1000.0;

    return this->is_in_range(timestamp, this->min_pretimestamp_, this->max_pretimestamp_);
  }
  else
  {
    return true;
  }

}

bool ClusterManager::is_in_range(double timestamp, double min_base_timestamp, double max_base_timestamp)
{
  int cnt = 0;

  std::cerr << "timestamp : " << timestamp << "\n";
  std::cerr << "min_pretimestamp : " << min_base_timestamp << "\n";
  std::cerr << "max_pretimestamp : " << min_base_timestamp << "\n";

  while (timestamp > max_base_timestamp)
  {
    max_base_timestamp += this->max_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * ++cnt;
  }

  min_base_timestamp += this->min_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * cnt;

  std::cerr << "min_current_timestamp : " << min_base_timestamp << "\n";
  std::cerr << "max_current_timestamp : " << min_base_timestamp << "\n";

  if (timestamp > min_base_timestamp)
  {
    this->min_pretimestamp_ = timestamp;
    this->max_pretimestamp_ = timestamp;
    return true;
  }
  else
  {
    return false;
  }
}