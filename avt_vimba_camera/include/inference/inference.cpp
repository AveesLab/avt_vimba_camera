#include "inference/inference.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


Darknet::Darknet(float thresh, char *cfg_file, char *weight_file)
{
    demo_thresh = thresh;

    net = parse_network_cfg_custom(cfg_file, 1, 1);    // set batch=1

    sleep(1);

    if(weight_file){
        load_weights(&net, weight_file);
    }

    sleep(1);

    net.benchmark_layers = 0;

    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);
    srand(2222222);

    l = net.layers[net.n-1];

    demo_classes = l.classes;
    printf("l.classes : %d\n", l.classes);

    int i;
    for (i = 0; i < net.n; ++i) {
        layer lc = net.layers[i];
        if (lc.type == YOLO) {
            lc.mean_alpha = 1.0 / avg_frames;
            l = lc;
        }
    }

    flag_exit = 0;
}

Darknet::~Darknet()
{
    // free memory
    // TODO : free memory, if pointers have data
    free_detections(dets, nboxes);
    free_network(net);
}

void Darknet::preprocess(cv::Mat& input_image)
{
    // cv::Mat to darknet::image
    this->det_s_ = get_image_from_mat(input_image, net.w, net.h, net.c);

    this->image_cols_ = input_image.cols;
    this->image_rows_ = input_image.rows;
}

void Darknet::inference()
{
    // inference
    float *X = this->det_s_.data;
    network_predict(net, X);
}

std::vector<ObjectDetection> Darknet::postprocess()
{
    const float nms = .45;    // 0.4F

    std::vector<ObjectDetection> result;

    dets = get_network_boxes(&net, net.w, net.h, demo_thresh, demo_thresh, 0, 1, &nboxes, 0);

    if (nms) {
        if (l.nms_kind == DEFAULT_NMS) do_nms_sort(dets, nboxes, l.classes, nms);
        else diounms_sort(dets, nboxes, l.classes, nms, l.nms_kind, l.beta_nms);
    }

    if (l.embedding_size) set_track_id(dets, nboxes, demo_thresh, l.sim_thresh, l.track_ciou_norm, l.track_history_size, l.dets_for_track, l.dets_for_show);

    result = this->convert_detections();

    free_image(this->det_s_);
    free(dets);

    nboxes = 0;

    return result;
}

image Darknet::get_image_from_mat(cv::Mat& input_image, int width, int height, int channel)
{
    channel = channel ? channel : 3;

    cv::Mat new_image = cv::Mat(height, width, CV_8UC(channel));
    cv::resize(input_image, new_image, new_image.size(), 0, 0, cv::INTER_LINEAR);
    if (channel > 1)
    {
        cv::cvtColor(new_image, new_image, cv::COLOR_RGB2BGR);
    }

    image im = mat_to_image(new_image);
    return im;
}

std::vector<ObjectDetection> Darknet::convert_detections()  // 'dets' is pointer
{
    std::vector<ObjectDetection> detections;

    for (int i = 0; i < nboxes; i++)
    {
        for (int j = 0; j < demo_classes; j++) 
        {
            if (dets[i].prob[j] > demo_thresh)
            {
                ObjectDetection detection;

                detection.id = j;
                detection.center_x = static_cast<float>(image_cols_) * ((dets[i].bbox.x < 1) ? dets[i].bbox.x : 1);
                detection.center_y = static_cast<float>(image_rows_) * ((dets[i].bbox.y < 1) ? dets[i].bbox.y : 1);
                detection.width_half = static_cast<float>(image_cols_) * ((dets[i].bbox.w < 1) ? dets[i].bbox.w / 2.0 : 0.5f);
                detection.height_half = static_cast<float>(image_rows_) * ((dets[i].bbox.h < 1) ? dets[i].bbox.h / 2.0 : 0.5f);
                detections.push_back(detection);

                break;
            }
        }
    }

    return detections;
}

image Darknet::mat_to_image(cv::Mat mat)
{
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();
    image im = make_image(w, h, c);
    unsigned char *data = (unsigned char *)mat.data;
    int step = mat.step;
    for (int y = 0; y < h; ++y) {
        for (int k = 0; k < c; ++k) {
            for (int x = 0; x < w; ++x) {
                //uint8_t val = mat.ptr<uint8_t>(y)[c * x + k];
                //uint8_t val = mat.at<Vec3b>(y, x).val[k];
                //im.data[k*w*h + y*w + x] = val / 255.0f;

                im.data[k*w*h + y*w + x] = data[y*step + x*c + k] / 255.0f;
            }
        }
    }
    return im;
}