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
  this->inference_ = std::make_shared<PoseNet>();
  
  // ROS2 : QoS
  const rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // ROS2 : Image Selection Synchronize
  this->cluster_synchronize_publisher_ = this->create_publisher<std_msgs::msg::Header>("/cluster/synchronize", system_qos);
  this->cluster_synchronize_subscriber_ = this->create_subscription<std_msgs::msg::Header>("/cluster/synchronize", system_qos, std::bind(&MonoCameraNode::ImageSelectionSynchronize, this, _1));

  // ROS2 : Pose Estimation Result
  this->poses_publisher_ = this->create_publisher<posenet_msgs::msg::Poses>("/cluster/poses", system_qos);

  // Image Selection : Synchronize
  this->synchronization_cnt_ = 0;
  this->cluster_flag_ = false;
  this->emergency_flag_ = false;
}

MonoCameraNode::~MonoCameraNode()
{
  cam_.stop();
}

void MonoCameraNode::ImageSelectionSynchronize(std_msgs::msg::Header::SharedPtr base_timestamp)
{
  if (std::stoi(base_timestamp->frame_id) == 0 && !this->emergency_flag_)
  {
    this->emergency_flag_ = true;
  }

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
  rclcpp::Time node_start_time = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "=== AVEES - Cluster-based Object Detection System with Scalable Performance for Autonomous Driving ===");

  sensor_msgs::msg::Image img;
  if (api_.frameToImage(vimba_frame_ptr, img))
  {
    // Set time stamp
    VmbUint64_t frame_timestamp;
    vimba_frame_ptr->GetTimestamp(frame_timestamp);
    img.header.stamp = rclcpp::Time(cam_.getTimestampRealTime(frame_timestamp) * 1.0e+9) + rclcpp::Duration(ptp_offset_, 0);
    img.header.frame_id = std::to_string(node_index_);

    if (this->cluster_flag_)
    {
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Cluster mode", this->node_index_);
    }
    else
    {
      if (this->node_index_ != 1)
      {
        RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Waiting mode", this->node_index_);
        return;
      }
    }

    // Image Selection
    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image timestamp : %lf sec.", this->node_index_, rclcpp::Time(img.header.stamp).seconds());
    if (this->image_selection_->IsSelfOrder(rclcpp::Time(img.header.stamp).seconds()) == true)
    {
      // Synchronization
      if (this->node_index_ == 1 && !this->cluster_flag_ && this->emergency_flag_)
      {
          std_msgs::msg::Header base_timestamp;
          base_timestamp.stamp = img.header.stamp;
          base_timestamp.frame_id = std::to_string(this->node_index_);
          this->cluster_synchronize_publisher_->publish(base_timestamp);
      }

      // RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Expect timestamp : %lf sec.", this->node_index_, this->image_selection_->GetEstimatedTimestamp());
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Take this image", this->node_index_);
    }
    else
    {
      // RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Expect timestamp : %lf sec.", this->node_index_, this->image_selection_->GetEstimatedTimestamp());
      RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Image selection : Drop this image", this->node_index_);
      return;  // terminate this iteration
    }

    // sensor_msgs::msg::image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    cv_bridge::CvImagePtr color_image_ptr(new cv_bridge::CvImage);
    cv::cvtColor(cv_ptr->image, color_image_ptr->image, cv::COLOR_BayerRG2RGB);
    color_image_ptr->header = cv_ptr->header;
    color_image_ptr->encoding = "bgr8";
  
    // Pose Estimation - DNN Inference
    posenet_msgs::msg::Poses poses;
    this->inference_->Inference(color_image_ptr->toImageMsg(), poses);
    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] Number of pose : %u", this->node_index_, poses.poses.size());

    // this->inference_->Showoverlay(color_image_ptr->image, poses);

    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] End of inference time : %lf", this->node_index_, this->get_clock()->now().seconds() - node_start_time.seconds());

    // Regular Output
    double computing_time = this->image_selection_->getComputingTime();
    double result_send_time = 0.02;  // Detections Message + End Message + Threshold
    double limit_time = computing_time - result_send_time;
    
    while ((limit_time) > (this->get_clock()->now().seconds() - node_start_time.seconds()))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Topic publish
    poses.header = img.header;
    this->poses_publisher_->publish(poses);

    RCLCPP_INFO(this->get_logger(), "[Computing Node %d] End of total time : %lf", this->node_index_, this->get_clock()->now().seconds() - node_start_time.seconds());
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Function frameToImage returned 0. No image published.");
  }

  RCLCPP_INFO(this->get_logger(), "======================================================================================================\n");
}

}  // namespace avt_vimba_camera
