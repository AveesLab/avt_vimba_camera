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

#ifndef MONO_CAMERA_H
#define MONO_CAMERA_H

#include "avt_vimba_camera/avt_vimba_camera.hpp"
#include "avt_vimba_camera/avt_vimba_api.hpp"

#include <avt_vimba_camera_msgs/srv/detail/load_settings__struct.hpp>
#include <avt_vimba_camera_msgs/srv/detail/save_settings__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>
#include "rtx_msg_interface/msg/bounding_box.hpp"
#include "rtx_msg_interface/msg/bounding_boxes.hpp"

// Cluster
#include "cluster_inference/cluster_inference.hpp"

// Inference
#include "dummy_inference/yolov7.hpp"

// benchmark
#include <string>
#include <ctime>
#include <fstream>


namespace avt_vimba_camera
{
class MonoCameraNode : public rclcpp::Node
{
public:
  MonoCameraNode();
  ~MonoCameraNode();
  void start();

private:
  // Cluster
  std::shared_ptr<ClusterManager> cluster_manager_;

  // Inference
  std::shared_ptr<Yolov7> dummy_inference_;

  AvtVimbaApi api_;
  AvtVimbaCamera cam_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  std::string frame_id_;
  bool use_measurement_time_;
  int32_t ptp_offset_;
  int32_t node_index_;
  bool image_crop_;

  int number_of_nodes_;
  int interval_;

  std::string inference_model_path_;

  // use sensor_msgs::msg::CompressedImage
  rclcpp::Publisher<rtx_msg_interface::msg::BoundingBoxes>::SharedPtr bounding_boxes_publisher_;

  void loadParams();
  void frameCallback(const FramePtr& vimba_frame_ptr);

  // benchmark
  bool use_benchmark_;
  std::fstream file_;
  void benchmark();
  void finish_benchmark();
};
}  // namespace avt_vimba_camera
#endif
