#include "monitor/demo.hpp"


MonitorDemo::MonitorDemo()
: Node("MonitorDemo")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using std::placeholders::_1;
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/raw_image", QOS_RKL10V, std::bind(&MonitorDemo::image_callback, this, _1));

  this->base_timestamp_.resize(this->number_of_nodes_);
  for (int & timestamp : this->base_timestamp_)
  {
    timestamp = 0;
  }

  this->can_receiver_ = std::make_shared<CanReceiver>();
  this->detections_per_node_.resize(this->number_of_nodes_);

  this->run_flag_ = true;

  // pthread
  pthread_mutex_init(&mutex, NULL);
  pthread_mutex_init(&mutex_image, NULL);
  pthread_cond_init(&cond, NULL);

  pthread_create(&thread_receive, NULL, receive_thread, this);
  pthread_create(&thread_show, NULL, show_thread, this);

  // Information
  RCLCPP_INFO(this->get_logger(), "[Demo] Finish initialization.");
}

MonitorDemo::~MonitorDemo()
{
  std::cerr << "[Demo] Press Ctrl+C to terminate system.\n";

  this->run_flag_ = false;

  pthread_mutex_destroy(&mutex);
  pthread_cond_destroy(&cond);

  pthread_join(thread_receive, NULL);
  pthread_join(thread_show, NULL);
}

void MonitorDemo::image_callback(const sensor_msgs::msg::Image::SharedPtr image)
{
  cv_bridge::CvImagePtr tmp_cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (tmp_cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(tmp_cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(tmp_cv_image->image, rgb8_image);

    tmp_cv_image->encoding = "rgb8";
  }

  // Append a image on queue
  pthread_mutex_lock(&mutex_image);
  this->image_queue_.push(tmp_cv_image);
  pthread_mutex_unlock(&mutex_image);
}

void MonitorDemo::can_receive()
{
  ObjectDetection detection;
  int node_index;

  node_index = this->can_receiver_->GetMessage(detection);

  if (detection.id == -1)
  {
    pthread_mutex_lock(&mutex);

    if (this->detections_per_node_[node_index].size())
    {
      this->base_timestamp_[node_index] = detection.time;

      std::vector<ObjectDetection>().swap(this->detections_);
      this->detections_per_node_[node_index].push_back(detection);
      this->detections_.swap(this->detections_per_node_[node_index]);
     
      pthread_cond_signal(&cond);
    }

    pthread_mutex_unlock(&mutex);
  }
  else if (detection.id == -2)
  {
    this->can_receiver_->SendMessage( this->base_timestamp_);
  }
  else
  {
    pthread_mutex_lock(&mutex);

    this->detections_per_node_[node_index].push_back(detection);

    pthread_mutex_unlock(&mutex);
  }

  usleep(700);
}

void MonitorDemo::can_show()
{
  pthread_mutex_lock(&mutex);

  pthread_cond_wait(&cond, &mutex);
  std::vector<ObjectDetection> detections;
  detections.swap(this->detections_);
  pthread_mutex_unlock(&mutex);

  // Image
  cv_bridge::CvImagePtr cv_image = nullptr;
  pthread_mutex_lock(&mutex_image);
  while (this->image_queue_.size())
  {
    int queued_image_stamp = static_cast<int>(static_cast<long long int>(rclcpp::Time(this->image_queue_.front()->header.stamp).seconds() * 1000.0) % 60000ll);

    if (queued_image_stamp != detections.back().time)
    {
      // consume image
      if (queued_image_stamp > detections.back().time + 30000)
      {
        this->image_queue_.pop();
      }
      if (queued_image_stamp < detections.back().time)
      {
        this->image_queue_.pop();
      }
      if ((queued_image_stamp > detections.back().time) && (queued_image_stamp < detections.back().time + 30000))
      {
        break;
      }
    }
    else
    {
      cv_image = this->image_queue_.front();

      this->image_queue_.pop();
    }
  }
  pthread_mutex_unlock(&mutex_image);

  // Check image
  if (cv_image == nullptr)
  {
    return;
  }

  // Draw bounding boxes
  draw_image(cv_image, detections);

  // Show image
  cv::imshow("Result image", cv_image->image);
  cv::waitKey(10);
}

void* MonitorDemo::receive_thread(void* arg)
{
  while (static_cast<MonitorDemo*>(arg)->run_flag_)
  {
    static_cast<MonitorDemo*>(arg)->can_receive();
  }
  return nullptr;
}

void* MonitorDemo::show_thread(void* arg)
{
  while (static_cast<MonitorDemo*>(arg)->run_flag_)
  {
    static_cast<MonitorDemo*>(arg)->can_show();
  }
  return nullptr;
}

void MonitorDemo::draw_image(cv_bridge::CvImagePtr cv_image, std::vector<ObjectDetection>& detections)
{
  for (size_t i = 0; i < (detections.size() - 1); i++) {
    // Get rectangle from 1 object
    cv::Rect r = cv::Rect(round(detections[i].center_x - detections[i].width_half),
                          round(detections[i].center_y - detections[i].height_half),
                          round(2 * detections[i].width_half),
                          round(2 * detections[i].height_half));

    // draw_box
    cv::rectangle(cv_image->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

    // put id
    cv::putText(cv_image->image, std::to_string(static_cast<int>(detections[i].id)), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
}
