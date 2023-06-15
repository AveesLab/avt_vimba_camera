#include "cluster/cluster.hpp"


ClusterManager::ClusterManager(int node_index, int number_of_nodes, double fps, double process_fps, double max_camera_cycle_time, double min_camera_cycle_time) : number_of_nodes_(number_of_nodes), fps_(fps), node_index_(node_index)
{
  this->base_timestamp_ = -1.0;

  this->pretimestamp_ = -1;

  this->frame_interval_ = fps / process_fps / this->number_of_nodes_;

  this->max_camera_cycle_time_ = max_camera_cycle_time;
  this->min_camera_cycle_time_ = min_camera_cycle_time;
}

ClusterManager::~ClusterManager()
{
}

void ClusterManager::register_base_timestamp(double timestamp)
{
  this->base_timestamp_ = timestamp * 1000.0;
}

bool ClusterManager::is_self_order(double timestamp)
{
  if (this->base_timestamp_ != -1.0)
  {
    timestamp *= 1000.0;

    if (this->pretimestamp_ != -1)
    {
      return this->is_in_range(timestamp, this->pretimestamp_, this->pretimestamp_);
    }
    else
    {
      double min_base_timestamp = this->base_timestamp_ + this->max_camera_cycle_time_ * this->frame_interval_ * this->node_index_;
      double max_base_timestamp = this->base_timestamp_ + this->min_camera_cycle_time_ * this->frame_interval_ * this->node_index_;
      return this->is_in_range(timestamp, min_base_timestamp, max_base_timestamp);
    }
  }
  else
  {
    return true;
  }

}

bool ClusterManager::is_in_range(double timestamp, double min_base_timestamp, double max_base_timestamp)
{
  int cnt = 0;

  while (timestamp > max_base_timestamp)
  {
    max_base_timestamp += this->max_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * ++cnt;
  }

  min_base_timestamp += this->min_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * cnt;

  if (timestamp > min_base_timestamp)
  {
    this->pretimestamp_ = timestamp;
    return true;
  }
  else
  {
    return false;
  }
}