#include "dummy_inference/yolov7.hpp"


using namespace nvinfer1;


Yolov7::Yolov7(std::string model_path) : model_path_(model_path)
{
    // Information
    std::cout << "[Inference] : Initialization Start." << std::endl;

    // Initialize model
    this->initialize_model(model_path);

    // Information
    std::cout << "[Inference] : Initialization Finish." << std::endl;
}

Yolov7::~Yolov7()
{
    // Release stream and buffers
    cudaStreamDestroy(this->stream_);
    CUDA_CHECK(cudaFree(this->device_buffers_[0]));
    CUDA_CHECK(cudaFree(this->device_buffers_[1]));
    delete[] this->output_buffer_host_;
    cuda_preprocess_destroy();

    // Destroy the engine
    delete this->context_;
    delete this->engine_;
    delete this->runtime_;

    // Information
    std::cout << "[Inference] : Destruction Finish." << std::endl;
}

void Yolov7::initialize_model(std::string model_path)
{
    // Deserialize the engine from file
    deserialize_engine(model_path, &runtime_, &engine_, &context_);

    CUDA_CHECK(cudaStreamCreate(&stream_));

    cuda_preprocess_init(kMaxInputImageSize);

    // Prepare cpu and gpu buffers
    prepare_buffer(engine_, &device_buffers_[0], &device_buffers_[1], &output_buffer_host_);
}

std::vector<ObjectDetection> Yolov7::get_detections(cv::Mat& image)
{
    std::vector<cv::Mat> image_batch;
    std::vector<std::vector<Detection>> result_batch;
    std::vector<ObjectDetection> detections;

    image_batch.push_back(image);

    // Inference
    result_batch = this->inference(image_batch);

    // Convert result_batch to detections
    detections = this->convert_detections(image, result_batch);

    return detections;
}

std::vector<std::vector<Detection>> Yolov7::inference(std::vector<cv::Mat>& image_batch)
{
    // Preprocess
    cuda_batch_preprocess(image_batch, device_buffers_[0], kInputW, kInputH, stream_);

    // Inference
    infer(*context_, stream_, (void**)device_buffers_, output_buffer_host_, kBatchSize);

    // Postprocess
    std::vector<std::vector<Detection>> result_batch;
    batch_nms(result_batch, output_buffer_host_, image_batch.size(), kOutputSize, kConfThresh, kNmsThresh);

    return result_batch;
}

std::vector<ObjectDetection> Yolov7::convert_detections(cv::Mat& image, std::vector<std::vector<Detection>>& result_batch)
{
    std::vector<ObjectDetection> detections;

    if (result_batch[0].size() != 0)
    {
        for (size_t i = 0; i < result_batch[0].size(); i++)
        {
            ObjectDetection detection;

            float l, r, t, b;
            float r_w = kInputW / (image.cols * 1.0);
            float r_h = kInputH / (image.rows * 1.0);

            if (r_h > r_w)
            {
                l = result_batch[0][i].bbox[0] - result_batch[0][i].bbox[2] / 2.f;
                r = result_batch[0][i].bbox[0] + result_batch[0][i].bbox[2] / 2.f;
                t = result_batch[0][i].bbox[1] - result_batch[0][i].bbox[3] / 2.f - (kInputH - r_w * image.rows) / 2;
                b = result_batch[0][i].bbox[1] + result_batch[0][i].bbox[3] / 2.f - (kInputH - r_w * image.rows) / 2;

                l = l / r_w;
                r = r / r_w;
                t = t / r_w;
                b = b / r_w;
            }
            else
            {
                l = result_batch[0][i].bbox[0] - result_batch[0][i].bbox[2] / 2.f - (kInputW - r_h * image.cols) / 2;
                r = result_batch[0][i].bbox[0] + result_batch[0][i].bbox[2] / 2.f - (kInputW - r_h * image.cols) / 2;
                t = result_batch[0][i].bbox[1] - result_batch[0][i].bbox[3] / 2.f;
                b = result_batch[0][i].bbox[1] + result_batch[0][i].bbox[3] / 2.f;

                l = l / r_h;
                r = r / r_h;
                t = t / r_h;
                b = b / r_h;
            }

            detection.id = result_batch[0][i].class_id;
            detection.center_x = (r + l) / 2.f;
            detection.center_y = (b + t) / 2.f;
            detection.width_half = (r - l) / 2.f;
            detection.height_half = (b - t) / 2.f;
            detections.push_back(detection);
        }
    }

    return detections;
}
