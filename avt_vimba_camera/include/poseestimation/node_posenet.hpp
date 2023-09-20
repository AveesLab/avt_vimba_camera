#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-inference/poseNet.h>

#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <posenet_msgs/msg/keypoints.hpp>
#include <posenet_msgs/msg/links.hpp>
#include <posenet_msgs/msg/objectpose.hpp>
#include <posenet_msgs/msg/poses.hpp>


class PoseNet
{
private:
	poseNet* net = NULL;
	uint32_t overlay_flags = poseNet::OVERLAY_DEFAULT;
	imageConverter* input_cvt   = NULL;
	imageConverter* overlay_cvt = NULL;

public:
    PoseNet();
    ~PoseNet();

    void Showoverlay( cv::Mat& input, posenet_msgs::msg::Poses& poses );
    bool Inference( const sensor_msgs::ImageConstPtr input, posenet_msgs::msg::Poses& poses );
};