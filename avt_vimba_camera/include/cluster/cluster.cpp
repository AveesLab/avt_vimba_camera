#include "cluster/cluster.hpp"


ClusterManager::ClusterManager(int node_index, int number_of_nodes, double fps, double process_fps, double max_camera_cycle_time, double min_camera_cycle_time) : number_of_nodes_(number_of_nodes), fps_(fps)
{
  this->cluster_nodes_.resize(this->number_of_nodes_);
  for (int & timestamp : this->cluster_nodes_)
  {
    timestamp = -1;
  }

  this->pretimestamp_ = -1;

  this->frame_interval_ = fps / process_fps / this->number_of_nodes_;

  this->max_camera_cycle_time_ = max_camera_cycle_time;
  this->min_camera_cycle_time_ = min_camera_cycle_time;
}

ClusterManager::~ClusterManager()
{
  this->cluster_nodes_.clear();
}

void ClusterManager::register_base_timestamp(int index, int timestamp)
{
  this->cluster_nodes_[index] = timestamp;
}

bool ClusterManager::is_self_order(double timestamp)
{
  int remaked_timestamp = static_cast<int>(timestamp * 1000.0) % 60000;
  bool exist_flag = false;

  if (this->pretimestamp_ == -1)
  {
    for (size_t i = 0; i < this->cluster_nodes_.size(); i++)
      if (this->cluster_nodes_[i] != -1)
      {
        exist_flag = true;

        if (this->is_in_range(remaked_timestamp, this->cluster_nodes_[i]))
        {
          return false;
        }
      }

    if (exist_flag == false)
    {
      this->pretimestamp_ = remaked_timestamp;
      return true;
    }
    else
    {
      if (this->is_set_timestamp(remaked_timestamp))
      {
        this->pretimestamp_ = remaked_timestamp;
        return true;
      }
    }
  }
  else
  {
    if (this->is_in_range(remaked_timestamp, this->pretimestamp_))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

bool ClusterManager::is_in_range(int remaked_timestamp, int & reserved_timestamp)
{
  int origin_remaked_timestamp = remaked_timestamp;
  if (remaked_timestamp < reserved_timestamp)
  {
    remaked_timestamp += 60000;
  }

  double max_predict_timestamp = reserved_timestamp;
  double min_predict_timestamp = reserved_timestamp;

  int cnt = 0;

  while (remaked_timestamp > max_predict_timestamp)
  {
    max_predict_timestamp += this->max_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * ++cnt;
  }

  min_predict_timestamp += this->min_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * cnt;

  if (remaked_timestamp > min_predict_timestamp)
  {
    reserved_timestamp = origin_remaked_timestamp;
    return true;
  }
  else
  {
    return false;
  }
}

bool ClusterManager::is_set_timestamp(int remaked_timestamp)
{
  for (size_t i = 0; i < this->cluster_nodes_.size(); i++)
  {
    if (this->cluster_nodes_[i] != -1)
    {
      int tmp_remaked_timestamp = remaked_timestamp;

      if (remaked_timestamp < this->cluster_nodes_[i])
      {
        tmp_remaked_timestamp = remaked_timestamp + 60000;
      }
      int reserved_timestamp = this->cluster_nodes_[i];

      int cnt = 0;

      double max_remaked_timestamp = tmp_remaked_timestamp;
      while (max_remaked_timestamp > reserved_timestamp)
      {
        max_remaked_timestamp -= this->max_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * ++cnt;
      }

      double min_predict_timestamp = tmp_remaked_timestamp - this->min_camera_cycle_time_ * this->frame_interval_ * this->number_of_nodes_ * cnt;

      int cnt2 = 0;

      double tmp_reserved_timestamp = reserved_timestamp;
      while (min_predict_timestamp < reserved_timestamp)
      {
        reserved_timestamp -= this->min_camera_cycle_time_ * this->frame_interval_ * ++cnt2;
      }

      double max_predict_timestamp = tmp_reserved_timestamp - this->max_camera_cycle_time_ * this->frame_interval_ * cnt2;

      if (max_remaked_timestamp < max_predict_timestamp)
      {
        return true;
      }
    }
  }

  return false;
}