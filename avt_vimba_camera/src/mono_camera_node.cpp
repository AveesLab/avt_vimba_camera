/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/mono_camera_node.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std::placeholders;

namespace avt_vimba_camera
{
MonoCameraNode::MonoCameraNode() : Node("camera"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
  this->LoadParams();

  // GigE Camera : Frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::FrameCallback, this, _1));

  // Image Selection
  this->image_selection_ = std::make_shared<ImageSelection>(this->node_index_, this->number_of_nodes_, this->local_inference_fps_, this->timestamp_margin_milisecond_);

  // Object Detection
  this->inference_ = std::make_shared<Darknet>(0.2, const_cast<char*>(dnn_cfg_path_.c_str()), const_cast<char*>(dnn_weight_path_.c_str()));

  // CAN
  this->can_ = std::make_shared<CanSender>(this->node_index_, this->can_send_time_interval_microsecond_);
  
  // ROS2 : QoS
  const rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // ROS2 : Image Selection Synchronize
  this->cluster_synchronize_publisher_ = this->create_publisher<std_msgs::msg::Header>("/cluster/synchronize", system_qos);
  this->cluster_synchronize_subscriber_ = this->create_subscription<std_msgs::msg::Header>("/cluster/synchronize", system_qos, std::bind(&MonoCameraNode::ImageSelectionSynchronize, this, _1));

  // Image Selection : Synchronize
  this->synchronization_cnt_ = 0;
  this->cluster_flag_ = false;

  // Benchmark
  time_t raw_time;
  struct tm* pTime_info;

  raw_time = time(NULL);
  pTime_info = localtime(&raw_time);

  std::string simulation_time = std::to_string(pTime_info->tm_mon + 1) + "_" + std::to_string(pTime_info->tm_mday) + "_" + std::to_string(pTime_info->tm_hour) + "_" + std::to_string(pTime_info->tm_min) + "_" + std::to_string(pTime_info->tm_sec);
  std::string directory = "/home/avees/ros2_ws/data/computing/Computing{NODE_ID}" + simulation_time + ".csv";

  this->file_.open(directory.c_str(), std::ios_base::out | std::ios_base::app);

  this->file_ << "timestamp,startpoint,aftergetimage,ismyframe,aftercluster,afterpreprocess,afterinference,afterpostprocess,numberofobject,afterpublish,endpoint\n";
}

MonoCameraNode::~MonoCameraNode()
{
  this->file_.close();
  RCLCPP_INFO(this->get_logger(), "Saving benchmark result is successful.");

  cam_.stop();
}

void MonoCameraNode::ImageSelectionSynchronize(std_msgs::msg::Header::SharedPtr base_timestamp)
{
  try {
    if (std::stoi(base_timestamp->frame_id) == 1)  // get only node 1's timestamp 
    {
      this->image_selection_->RegisterBaseTimestamp(rclcpp::Time(base_timestamp->stamp).seconds());
      this->cluster_flag_ = true;
    }
  }
  catch (int error)
  {
    std::cerr << "image synchronize index failure";
    return;
  }
}

void MonoCameraNode::LoadParams()
{
  ip_ = this->declare_parameter("ip", "");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "");
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);

  // Cluster Information
  node_index_ = this->declare_parameter("node_index", 0);

  // Image Selection
  number_of_nodes_ = this->declare_parameter("number_of_nodes", 1);
  local_inference_fps_ = this->declare_parameter("local_inference_fps", 5.0);
  timestamp_margin_milisecond_ = this->declare_parameter("timestamp_margin_milisecond", 0.05);

  // Image Selection : Synchronize
  convert_frame_ = this->declare_parameter("convert_frame", 1);

  // Object Detection
  dnn_cfg_path_ = this->declare_parameter("dnn_cfg_path", "/home/avees/ros2_ws/weights/yolov4-p6.cfg");
  dnn_weight_path_ = this->declare_parameter("dnn_weight_path", "/home/avees/ros2_ws/weights/yolov4-p6.weights");

  // CAN
  can_send_time_interval_microsecond_ = this->declare_parameter("can_send_time_interval_microsecond", 500);

  RCLCPP_INFO(this->get_logger(), "[Initialize] Parameters loaded");
}

void MonoCameraNode::Start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

void MonoCameraNode::FrameCallback(const FramePtr& vimba_frame_ptr)
{
  this->bench_startpoint = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);

  RCLCPP_INFO(this->get_logger(), "=== AVEES - Cluster-based Object Detection System with Scalable Performance for Autonomous Driving ===");

  rclcpp::Time node_start_time = this->get_clock()->now();

  sensor_msgs::msg::Image img;
  if (api_.frameToImage(vimba_frame_ptr, img))
  {
    // Set time stamp
    VmbUint64_t frame_timestamp;
    vimba_frame_ptr->GetTimestamp(frame_timestamp);
    img.header.stamp = rclcpp::Time(cam_.getTimestampRealTime(frame_timestamp) * 1.0e+9) + rclcpp::Duration(ptp_offset_, 0);

    if (this->cluster_flag_)
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Cluster mode", this->node_index_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Local mode", this->node_index_);
    }

    this->bench_timestamp = static_cast<long long int>(rclcpp::Time(img.header.stamp).seconds() * 1000000.0);
    this->bench_aftergetimage = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);

    // Image Selection
    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image timestamp : %lf sec.", this->node_index_, rclcpp::Time(img.header.stamp).seconds());
    this->bench_ismyframe = 1ll;
    if (this->image_selection_->IsSelfOrder(rclcpp::Time(img.header.stamp).seconds()) == true)
    {
      // Synchronization
      if (this->node_index_ == 1)
      {
        if (this->synchronization_cnt_ == this->convert_frame_)
        {
          std_msgs::msg::Header base_timestamp;
          base_timestamp.stamp = img.header.stamp;
          base_timestamp.frame_id = std::to_string(this->node_index_);
          this->cluster_synchronize_publisher_->publish(base_timestamp);
        }
        else if (this->synchronization_cnt_ < this->convert_frame_)
        {
          this->synchronization_cnt_ += 1;
        }
      }

      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Expect timestamp : %lf sec.", this->node_index_, this->image_selection_->GetEstimatedTimestamp());
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Take this image", this->node_index_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Expect timestamp : %lf sec.", this->node_index_, this->image_selection_->GetEstimatedTimestamp());
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Drop this image", this->node_index_);
      this->bench_ismyframe = 0ll;
      this->file_ << bench_timestamp << "," << bench_startpoint << "," << bench_aftergetimage << "," << bench_ismyframe << "\n";
      return;  // terminate this iteration
    }

    this->bench_aftercluster = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);

    // sensor_msgs::msg::image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    cv::Mat color_image;
    cv::cvtColor(cv_ptr->image, color_image, cv::COLOR_BayerRG2RGB);

    // Object Detection - Preprocess
    this->inference_->Preprocess(color_image);

    this->bench_afterpreprocess = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);
    
    // Object Detection - DNN Inference
    this->inference_->Inference();

    this->bench_afterinference = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);

    // Object Detection - Postprocess
    std::vector<ObjectDetection> detections = this->inference_->Postprocess();

    this->bench_afterpostprocess = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);

    // CAN
    this->can_->WriteMessages(rclcpp::Time(img.header.stamp).seconds(), detections);

    this->bench_numberofobject = static_cast<long long int>(detections.size());
    this->bench_afterpublish = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Function frameToImage returned 0. No image published.");
  }

  RCLCPP_INFO(this->get_logger(), "======================================================================================================\n");

  this->bench_endpoint = static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0);
  this->file_ << bench_timestamp << "," << bench_startpoint << "," << bench_aftergetimage << "," << bench_ismyframe << "," << bench_aftercluster << "," << bench_afterpreprocess << "," << bench_afterinference << "," << bench_afterpostprocess << "," << bench_numberofobject << "," << bench_afterpublish << "," << bench_endpoint << "\n";
}

}  // namespace avt_vimba_camera
