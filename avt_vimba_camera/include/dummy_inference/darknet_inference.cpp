#include "dummy_inference/darknet_inference.hpp"


Darknet::Darknet(thresh, cfg_file, weight_file)
{
    demo_thresh = thresh;

    net = parse_network_cfg_custom(cfg_file, 1, 1);    // set batch=1
    if(weightfile){
        load_weights(&net, weight_file);
    }

    net.benchmark_layers = benchmark_layers;

    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);
    srand(2222222);

    layer l = net.layers[net.n-1];
    int j;

    cv_images = (mat_cv**)xcalloc(avg_frames, sizeof(mat_cv));

    int i;
    for (i = 0; i < net.n; ++i) {
        layer lc = net.layers[i];
        if (lc.type == YOLO) {
            lc.mean_alpha = 1.0 / avg_frames;
            l = lc;
        }
    }

    if (l.classes != demo_classes) {
        printf("\n Parameters don't match: in cfg-file classes=%d, in data-file classes=%d \n", l.classes, demo_classes);
        exit(0);
    }

    flag_exit = 0;
}

Darknet::~Darknet()
{
    // free memory
    // TODO : free memory, if pointers have data
    free_detections(dets, nboxes);

    free_ptrs((void **)names, net.layers[net.n - 1].classes);

    free_network(net);
}

std::vector<ObjectDetection> Darknet::get_detections(cv::Mat& input_image)
{
    this->inference(input_image);

    return this->convert_detections(input_image);
}

void Darknet::inference(cv::Mat& input_image)
{
    const float nms = .45;    // 0.4F

    // cv::Mat to darknet::image
    image det_s = get_image_from_mat(input_image, net.w, net.h, net.c);

    // inference
    layer l = net.layers[net.n - 1];
    float *X = det_s.data;
    network_predict(net, X);

    dets = this->get_network_boxes(&net, net.w, net.h, demo_thresh, demo_thresh, 0, 1, &nboxes, 0);

    if (nms) {
        if (l.nms_kind == DEFAULT_NMS) do_nms_sort(dets, nboxes, l.classes, nms);
        else diounms_sort(dets, nboxes, l.classes, nms, l.nms_kind, l.beta_nms);
    }

    if (l.embedding_size) set_track_id(dets, nboxes, demo_thresh, l.sim_thresh, l.track_ciou_norm, l.track_history_size, l.dets_for_track, l.dets_for_show);
    free_detections(dets, nboxes);
}

image Darknet::get_image_from_mat(cv::Mat& image, int width, int height, int channel)
{
    channel = channel ? channel : 3;

    cv::Mat new_image = cv::Mat(height, width, CV_8UC(channel));
    cv::resize(image, new_image, new_image.size(), 0, 0, cv::INTER_LINEAR);
    if (c > 1)
    {
        cv::cvtColor(new_image, new_image, cv::COLOR_RGB2BGR);
    }

    image im = mat_to_image(new_image);
    return im;
}

std::vector<ObjectDetection> Darknet::convert_detections(cv::Mat& image)  // 'dets' is static pointer
{
    std::vector<ObjectDetection> detections;

    for (int i = 0; i < nboxes; i++)
    {
        ObjectDetection detection;

        image.cols;
        image.rows;

        detection.id = dets[i].track_id;
        detection.center_x = static_cast<float>(img.cols) * ((dets[i].bbox.x < 1) dets[i].bbox.x : 1);
        detection.center_y = static_cast<float>(img.rows) * ((dets[i].bbox.y < 1) dets[i].bbox.y : 1);
        detection.width_half = static_cast<float>(img.cols) * ((dets[i].bbox.w < 1) dets[i].bbox.w / 2.0 : 0.5f);
        detection.height_half = static_cast<float>(img.rows) * ((dets[i].bbox.h < 1) dets[i].bbox.h / 2.0 : 0.5f);
        detections.push_back(detection);
    }


    return detections;
}