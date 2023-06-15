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

#include <thread>
#include <cmath>

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
  loadParams();

  // Cluster
  this->cluster_manager_ = std::make_shared<ClusterManager>(static_cast<int>(this->node_index_), this->number_of_nodes_, this->camera_fps_, this->inference_fps_, this->max_camera_cycle_time_, this->min_camera_cycle_time_);

  // Inference
  this->dummy_inference_ = std::make_shared<Darknet>(0.2, const_cast<char*>(inference_cfg_path_.c_str()), const_cast<char*>(inference_weight_path_.c_str()));

  // Can Sender
  this->pcan_sender_ = std::make_shared<ObjectDetectionsSender>(this->can_id_, this->time_interval_);

  // QoS
  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  // Set the result publisher
  this->bounding_boxes_publisher_ = this->create_publisher<rtx_msg_interface::msg::BoundingBoxes>("/cluster/result", QOS_RKL10V);

  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::frameCallback, this, _1));

  // Set the result publisher
  this->cluster_synchronize_publisher_ = this->create_publisher<std_msgs::msg::Header>("/cluster/synchronize", QOS_RKL10V);
  this->cluster_synchronize_subscriber_ = this->create_subscription<std_msgs::msg::Header>("/cluster/synchronize", QOS_RKL10V, std::bind(&MonoCameraNode::ClusterSynchronize, this, _1));

  this->cnt_ = 0;

  this->cluster_flag_ = false;

  this->benchmark();

  if (pcan_benchmark_)
  {
    this->pcan_sender_->SetBenchmark(pcan_benchmark_start_stamp_, pcan_benchmark_stamp_interval_);
  }

  std_msgs::msg::Header initializer;
  initializer.stamp = rclcpp::Time(0);
  initializer.frame_id = this->node_index_;
  this->cluster_synchronize_publisher_->publish(initializer); // ros2 don't receive first data
}

MonoCameraNode::~MonoCameraNode()
{
  this->dummy_inference_.reset();

  this->cluster_manager_.reset();

  cam_.stop();
  //camera_info_pub_.shutdown();

  this->finish_benchmark();
}

void MonoCameraNode::ClusterSynchronize(std_msgs::msg::Header::SharedPtr time)
{
  this->cluster_manager_->register_base_timestamp(rclcpp::Time(time->stamp).seconds());
  this->cluster_flag_ = true;
}

void MonoCameraNode::loadParams()
{
  ip_ = this->declare_parameter("ip", "");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "");
  use_measurement_time_ = this->declare_parameter("use_measurement_time", false);
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);
  image_crop_ = this->declare_parameter("image_crop", false);

  // Cluster
  node_index_ = this->declare_parameter("node_index", 0);
  number_of_nodes_ = this->declare_parameter("number_of_nodes", 1);
  camera_fps_ = this->declare_parameter("camera_fps", 30.0);
  inference_fps_ = this->declare_parameter("inference_fps", 5.0);
  max_camera_cycle_time_ = this->declare_parameter("max_camera_cycle_time", 33.5);
  min_camera_cycle_time_ = this->declare_parameter("min_camera_cycle_time", 33.0);
  convert_frame_ = this->declare_parameter("convert_frame", 10);

  // Inference
  inference_model_path_ = this->declare_parameter("inference_model_path", "/home/avees/ros2_ws/weights/yolov7.engine");
  inference_cfg_path_ = this->declare_parameter("inference_cfg_path", "/home/avees/ros2_ws/weights/yolov4-p6.cfg");
  inference_weight_path_ = this->declare_parameter("inference_weight_path", "/home/avees/ros2_ws/weights/yolov4-p6.weights");

  // Pcan
  use_can_ = this->declare_parameter("use_can", true);
  can_id_ = this->declare_parameter("can_id", 101);
  time_interval_ = this->declare_parameter("time_interval", 500);

  // Pcan
  pcan_benchmark_ = this->declare_parameter("pcan_benchmark", false);
  pcan_benchmark_start_stamp_ = this->declare_parameter("pcan_benchmark_start_stamp", 0.0);
  pcan_benchmark_stamp_interval_ = this->declare_parameter("pcan_benchmark_stamp_interval", 0.0);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded");
}

void MonoCameraNode::start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

void MonoCameraNode::frameCallback(const FramePtr& vimba_frame_ptr)
{
  rclcpp::Time ros_time = this->get_clock()->now();

  sensor_msgs::msg::Image img;
  if (api_.frameToImage(vimba_frame_ptr, img))
  {
    // Set time stamp
    if (use_measurement_time_)
    {
      VmbUint64_t frame_timestamp;
      vimba_frame_ptr->GetTimestamp(frame_timestamp);
      img.header.stamp = rclcpp::Time(cam_.getTimestampRealTime(frame_timestamp) * 1.0e+9) + rclcpp::Duration(ptp_offset_, 0);

      RCLCPP_INFO(this->get_logger(), "capture time : %lf sec.", cam_.getTimestampRealTime(frame_timestamp) + rclcpp::Duration(ptp_offset_, 0).seconds());
    }
    else
    {
      img.header.stamp = ros_time;
    }

    // Set frame_id (= node index)
    img.header.frame_id = this->node_index_;

    // benchmark
    if (use_benchmark_) {
      // image time stamp
      this->file_ << static_cast<long long int>(rclcpp::Time(img.header.stamp).seconds() * 1000000.0) << ",";

      // node start point
      this->file_ << static_cast<long long int>(ros_time.seconds() * 1000000.0) << ",";

      // after get image
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }

    // Cluster
    if (this->cluster_manager_->is_self_order(rclcpp::Time(img.header.stamp).seconds()) == false)
    {
      // benchmark
      if (use_benchmark_) {
        this->file_ << static_cast<long long int>(0) << "\n";
      }

      return;
    }
    else
    {
      if (this->node_index_ == 0)
      {
        this->cnt_ += 1;

        if (this->cnt_ == this->convert_frame_)
        {
          std_msgs::msg::Header initializer;
          initializer.stamp = img.header.stamp;
          initializer.frame_id = this->node_index_;
          this->cluster_synchronize_publisher_->publish(initializer);
        }
      }

      // benchmark
      if (use_benchmark_) {
        this->file_ << static_cast<long long int>(1) << ",";
      }

      if (this->cluster_flag_)
      {
        RCLCPP_INFO(this->get_logger(), "Cluster Sorted");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Cluster Non-sorted");
      }
    }

    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }

    // sensor_msgs::msg::image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    cv_bridge::CvImage cv_bridge;
    cv::Mat color_image;

    cv::cvtColor(cv_ptr->image, color_image, cv::COLOR_BayerRG2RGB);

    // Image Crop
    if (image_crop_)
    {
      RCLCPP_INFO(this->get_logger(), "Image crop to 640x480");

      cv::Mat resized_image;
      cv::resize(color_image, resized_image, cv::Size(640,480));

      cv_bridge = cv_bridge::CvImage(img.header, sensor_msgs::image_encodings::RGB8, resized_image);
    }
    else
    {
      cv_bridge = cv_bridge::CvImage(img.header, sensor_msgs::image_encodings::RGB8, color_image);
    }

    // Preprocess
    this->dummy_inference_->preprocess(cv_bridge.image);

    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }

    // Inference
    this->dummy_inference_->inference();

    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }

    // Postprocess
    std::vector<ObjectDetection> detections = this->dummy_inference_->postprocess();

    int tmp_number_of_object = static_cast<int>(detections.size());

    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }

    VmbUint64_t frame_ID;
    vimba_frame_ptr->GetFrameID(frame_ID);
    RCLCPP_INFO(this->get_logger(), "Frame ID : %d .", frame_ID);

    if (use_can_)
    {
      // Can send
      if ((this->pcan_benchmark_) && (rclcpp::Time(img.header.stamp).seconds() > this->pcan_benchmark_start_stamp_))
      {
        tmp_number_of_object = this->pcan_sender_->WriteMessagesWithBenchmark(rclcpp::Time(img.header.stamp).seconds(), detections);
        RCLCPP_INFO(this->get_logger(), "[Benchmark] Number of object : %d .", tmp_number_of_object);
      }
      else
      {
        this->pcan_sender_->WriteMessages(rclcpp::Time(img.header.stamp).seconds(), detections);
      }

      RCLCPP_INFO(this->get_logger(), "Can send.");
    }
    else
    {
      // Ethernet send : rtx_msgs publish
      rtx_msg_interface::msg::BoundingBoxes bounding_box_message;
      bounding_box_message.image_header = img.header;
      bounding_box_message.header.stamp = ros_time;
      bounding_box_message.header.frame_id = static_cast<int>(frame_ID);

      if (detections.size() != 0)
      {
        for(size_t i = 0; i < detections.size(); i++)
        {
          rtx_msg_interface::msg::BoundingBox bounding_box;

          bounding_box.left = static_cast<float>(detections[i].center_x);
          bounding_box.right = static_cast<float>(detections[i].center_y);
          bounding_box.top = static_cast<float>(detections[i].width_half);
          bounding_box.bot = static_cast<float>(detections[i].height_half);
          bounding_box.id  = static_cast<float>(detections[i].id);
          bounding_box_message.bounding_boxes.push_back(bounding_box);
        }
      }

      this->bounding_boxes_publisher_->publish(bounding_box_message);

      RCLCPP_INFO(this->get_logger(), "Publish.");
    }

    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<int>(tmp_number_of_object) << ",";
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Function frameToImage returned 0. No image published.");
  }

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << "\n";
  }
}

void MonoCameraNode::benchmark()
{
  use_benchmark_ = this->declare_parameter("use_benchmark", false);

  if (use_benchmark_)
  {
    time_t raw_time;
    struct tm* pTime_info;

    raw_time = time(NULL);
    pTime_info = localtime(&raw_time);

    std::string simulation_time = std::to_string(pTime_info->tm_mon + 1) + "_" + std::to_string(pTime_info->tm_mday) + "_" + std::to_string(pTime_info->tm_hour) + "_" + std::to_string(pTime_info->tm_min) + "_" + std::to_string(pTime_info->tm_sec);
    std::string directory = "./data/avt_vimba_camera/" + simulation_time + ".csv";

    this->file_.open(directory.c_str(), std::ios_base::out | std::ios_base::app);

    this->file_ << "timestamp,startpoint,aftergetimage,ismyframe,aftercluster,afterpreprocess,afterinference,afterpostprocess,numberofobject,afterpublish,endpoint\n";
  }
}

void MonoCameraNode::finish_benchmark()
{
  if (use_benchmark_)
  {
    this->file_.close();
    RCLCPP_INFO(this->get_logger(), "Saving benchmark result is successful.");
  }
}

}  // namespace avt_vimba_camera