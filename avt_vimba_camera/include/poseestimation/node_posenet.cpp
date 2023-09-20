#include "poseestimation/node_posenet.hpp"


PoseNet::PoseNet()
{
	net = NULL;
	overlay_flags = poseNet::OVERLAY_DEFAULT;
	input_cvt   = NULL;
	overlay_cvt = NULL;

	std::string model_name  = "resnet18-body";
	std::string model_path;
	float threshold = POSENET_DEFAULT_THRESHOLD;

	/*
	* load object detection network
	*/
	if( model_path.size() > 0 )
	{
		// create network using custom model paths
		net = poseNet::Create(model_name.c_str(), threshold, DEFAULT_MAX_BATCH_SIZE, TYPE_INT8, DEVICE_GPU, true );
	}
	else
	{
		// create network using the built-in model
		net = poseNet::Create(model_name.c_str());
	}

	if( !net )
	{
		ROS_ERROR("failed to load detectNet model");
		return;
	}

	/*
	* create image converter objects
	*/
	input_cvt = new imageConverter();
	overlay_cvt = new imageConverter();

	if( !input_cvt || !overlay_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return;
	}
}

PoseNet::~PoseNet()
{
	/*
	* free resources
	*/
	delete net;
	delete input_cvt;
	delete overlay_cvt;
}

// publish overlay image
void PoseNet::Showoverlay( cv::Mat& input, posenet_msgs::msg::Poses& poses )
{
	// get the image dimensions
	const float line_width = std::max(std::max(input.cols, input.rows) * 0.0013f, 1.5f);
    const float circle_radius = std::max(std::max(input.cols, input.rows) * 0.0052f, 4.0f);

	for (const auto& pose : poses.poses)
    {
        if (overlay_flags & poseNet::OVERLAY_BOX)
        {
            cv::rectangle(input, cv::Rect(pose.left, pose.top, pose.right - pose.left, pose.bottom - pose.top),
                          cv::Scalar(255, 255, 255), 1);
        }

        if (overlay_flags & poseNet::OVERLAY_LINKS)
        {
            for (const auto& link : pose.links)
            {
                const int a = link.first;
                const int b = link.second;
                cv::line(input, cv::Point(pose.keypoints[a].x, pose.keypoints[a].y), cv::Point(pose.keypoints[b].x, pose.keypoints[b].y), cv::Scalar(0, 255, 0), line_width);
            }
        }

        if (overlay_flags & poseNet::OVERLAY_KEYPOINTS)
        {
            for (const auto& keypoint : pose.keypoints)
            {
                cv::circle(input, cv::Point(keypoint.x, keypoint.y), circle_radius, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

	cv::imshow("Overlay Image", input);
    cv::waitKey(1);
}


// input image subscriber callback
bool PoseNet::Inference( const sensor_msgs::ImageConstPtr input, posenet_msgs::msg::Poses& poses )
{
	// convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_ERROR("failed to conver image");
		return false;
	}

	std::vector<poseNet::ObjectPose> poses_vector;

	if( !net->Process(input_cvt->ImageGPU(), input->width, input->height, poses_vector, poseNet::OVERLAY_NONE))
	{
		ROS_ERROR("failed to inference image");
		return false;
	}

	if( poses_vector.size() > 0 )
	{
		for ( size_t index = 0; index < poses_vector.size(); index++)
		{
			posenet_msgs::msg::Objectpose tmp_objectpose;
			tmp_objectpose.id = poses_vector[index].ID;
			tmp_objectpose.left = poses_vector[index].Left;
			tmp_objectpose.right = poses_vector[index].Right;
			tmp_objectpose.top = poses_vector[index].Top;
			tmp_objectpose.bottom = poses_vector[index].Bottom;

			posenet_msgs::msg::Keypoints tmp_keypoints;
			for (size_t keypoints_index = 0; keypoints_index < poses_vector[index].Keypoints.size(); keypoints_index++)
			{
				tmp_keypoints.id = poses_vector[index].Keypoints[keypoints_index].ID;
				tmp_keypoints.x = poses_vector[index].Keypoints[keypoints_index].x;
				tmp_keypoints.y = poses_vector[index].Keypoints[keypoints_index].y;
				tmp_objectpose.keypoints.push_back(tmp_keypoints);
			}

			posenet_msgs::msg::Links tmp_links;
			for (size_t Links_index = 0; Links_index < poses_vector[index].Links.size(); Links_index++)
			{
				tmp_links.first = poses_vector[index].Links[Links_index].at(0);
				tmp_links.second = poses_vector[index].Links[Links_index].at(1);
				tmp_objectpose.links.push_back(tmp_links);
			}

			poses.poses.push_back(tmp_objectpose);
		}
		return true;
	}

	return false;
}