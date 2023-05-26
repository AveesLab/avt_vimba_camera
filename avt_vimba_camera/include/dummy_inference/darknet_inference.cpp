#include "dummy_inference/darknet_inference.hpp"


Darknet::Darknet()
{
    if (avgframes < 1) avgframes = 1;
    avg_frames = avgframes;
    letter_box = letter_box_in;
    //skip = frame_skip;
    image **alphabet = load_alphabet();
    int delay = frame_skip;
    demo_names = names;
    demo_alphabet = alphabet;
    demo_classes = classes;
    demo_thresh = thresh;
    demo_ext_output = ext_output;
    demo_json_port = json_port;
    printf("Demo\n");
    net = parse_network_cfg_custom(cfgfile, 1, 1);    // set batch=1
    if(weightfile){
        load_weights(&net, weightfile);
    }
    if (net.letter_box) letter_box = 1;
    net.benchmark_layers = benchmark_layers;
    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);
    srand(2222222);

    if(filename){
        printf("video file: %s\n", filename);
        cap = get_capture_video_stream(filename);
        demo_skip_frame = is_live_stream(filename);
    }else{
        printf("Webcam index: %d\n", cam_index);
        cap = get_capture_webcam(cam_index);
        demo_skip_frame = true;
    }

    if (!cap) {
#ifdef WIN32
        printf("Check that you have copied file opencv_ffmpeg340_64.dll to the same directory where is darknet.exe \n");
#endif
        error("Couldn't connect to webcam.", DARKNET_LOC);
    }

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
        getchar();
        exit(0);
    }

    flag_exit = 0;
}

Darknet::~Darknet()
{
    // free memory
    free_image(in_s);
    free_detections(dets, nboxes);

    free_ptrs((void **)names, net.layers[net.n - 1].classes);

    const int nsize = 8;
    for (j = 0; j < nsize; ++j) {
        for (i = 32; i < 127; ++i) {
            free_image(alphabet[j][i]);
        }
        free(alphabet[j]);
    }

    free(alphabet);
    free_network(net);
}

std::vector<ObjectDetection> Darknet::get_detections(cv::Mat& image)
{

}

std::vector<std::vector<Detection>> Darknet::inference(cv::Mat& input_image)
{
    const float nms = .45;    // 0.4F

    // cv::Mat to darknet::image
    image det_s = get_image_from_mat(input_image, net.w, net.h, net.c);

    // inference
    layer l = net.layers[net.n - 1];
    float *X = det_s.data;
    network_predict(net, X);

    dets = get_network_boxes(&net, net.w, net.h, demo_thresh, demo_thresh, 0, 1, &nboxes, 0);

    if (nms) {
        if (l.nms_kind == DEFAULT_NMS) do_nms_sort(dets, nboxes, l.classes, nms);
        else diounms_sort(dets, nboxes, l.classes, nms, l.nms_kind, l.beta_nms);
    }

    if (l.embedding_size) set_track_id(dets, nboxes, demo_thresh, l.sim_thresh, l.track_ciou_norm, l.track_history_size, l.dets_for_track, l.dets_for_show);


}

image Darknet::get_image_from_mat(cv::Mat& image, int width, int height, int channel)
{
    channel = channel ? channel : 3;

    cv::Mat new_image = cv::Mat(height, width, CV_8UC(channel));
    cv::resize(image, new_image, new_image.size(), 0, 0, cv::INTER_LINEAR);
    if (c>1) cv::cvtColor(new_image, new_image, cv::COLOR_RGB2BGR);

    image im = mat_to_image(new_image);
    return im;
}

std::vector<ObjectDetection> Darknet::convert_detections(cv::Mat& image, // dets)
{
    try {
        int i, j;

        for (i = 0; i < num; ++i) {
            char labelstr[4096] = { 0 };
            int class_id = -1;
            for (j = 0; j < classes; ++j) {
                int show = strncmp(names[j], "dont_show", 9);
                if (dets[i].prob[j] > thresh && show) {
                    if (class_id < 0) {
                        strcat(labelstr, names[j]);
                        class_id = j;
                        char buff[20];
                        if (dets[i].track_id) {
                            sprintf(buff, " (id: %d)", dets[i].track_id);
                            strcat(labelstr, buff);
                        }
                        sprintf(buff, " (%2.0f%%)", dets[i].prob[j] * 100);
                        strcat(labelstr, buff);
                        printf("%s: %.0f%% ", names[j], dets[i].prob[j] * 100);
                        if (dets[i].track_id) printf("(track = %d, sim = %f) ", dets[i].track_id, dets[i].sim);
                    }
                    else {
                        strcat(labelstr, ", ");
                        strcat(labelstr, names[j]);
                        printf(", %s: %.0f%% ", names[j], dets[i].prob[j] * 100);
                    }
                }
            }
            if (class_id >= 0) {
                int width = std::max(1.0f, show_img->rows * .002f);

                //if(0){
                //width = pow(prob, 1./2.)*10+1;
                //alphabet = 0;
                //}

                //printf("%d %s: %.0f%%\n", i, names[class_id], prob*100);
                int offset = class_id * 123457 % classes;
                float red = get_color(2, offset, classes);
                float green = get_color(1, offset, classes);
                float blue = get_color(0, offset, classes);
                float rgb[3];

                //width = prob*20+2;

                rgb[0] = red;
                rgb[1] = green;
                rgb[2] = blue;
                box b = dets[i].bbox;
                if (std::isnan(b.w) || std::isinf(b.w)) b.w = 0.5;
                if (std::isnan(b.h) || std::isinf(b.h)) b.h = 0.5;
                if (std::isnan(b.x) || std::isinf(b.x)) b.x = 0.5;
                if (std::isnan(b.y) || std::isinf(b.y)) b.y = 0.5;
                b.w = (b.w < 1) ? b.w : 1;
                b.h = (b.h < 1) ? b.h : 1;
                b.x = (b.x < 1) ? b.x : 1;
                b.y = (b.y < 1) ? b.y : 1;
                //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

                int left = (b.x - b.w / 2.)*show_img->cols;
                int right = (b.x + b.w / 2.)*show_img->cols;
                int top = (b.y - b.h / 2.)*show_img->rows;
                int bot = (b.y + b.h / 2.)*show_img->rows;

                if (left < 0) left = 0;
                if (right > show_img->cols - 1) right = show_img->cols - 1;
                if (top < 0) top = 0;
                if (bot > show_img->rows - 1) bot = show_img->rows - 1;

                //int b_x_center = (left + right) / 2;
                //int b_y_center = (top + bot) / 2;
                //int b_width = right - left;
                //int b_height = bot - top;
                //sprintf(labelstr, "%d x %d - w: %d, h: %d", b_x_center, b_y_center, b_width, b_height);

                float const font_size = show_img->rows / 1000.F;
                cv::Size const text_size = cv::getTextSize(labelstr, cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, 1, 0);
                cv::Point pt1, pt2, pt_text, pt_text_bg1, pt_text_bg2;
                pt1.x = left;
                pt1.y = top;
                pt2.x = right;
                pt2.y = bot;
                pt_text.x = left;
                pt_text.y = top - 4;// 12;
                pt_text_bg1.x = left;
                pt_text_bg1.y = top - (3 + 18 * font_size);
                pt_text_bg2.x = right;
                if ((right - left) < text_size.width) pt_text_bg2.x = left + text_size.width;
                pt_text_bg2.y = top;
                cv::Scalar color;
                color.val[0] = red * 256;
                color.val[1] = green * 256;
                color.val[2] = blue * 256;

                // you should create directory: result_img
                //static int copied_frame_id = -1;
                //static IplImage* copy_img = NULL;
                //if (copied_frame_id != frame_id) {
                //    copied_frame_id = frame_id;
                //    if(copy_img == NULL) copy_img = cvCreateImage(cvSize(show_img->width, show_img->height), show_img->depth, show_img->nChannels);
                //    cvCopy(show_img, copy_img, 0);
                //}
                //static int img_id = 0;
                //img_id++;
                //char image_name[1024];
                //sprintf(image_name, "result_img/img_%d_%d_%d_%s.jpg", frame_id, img_id, class_id, names[class_id]);
                //CvRect rect = cvRect(pt1.x, pt1.y, pt2.x - pt1.x, pt2.y - pt1.y);
                //cvSetImageROI(copy_img, rect);
                //cvSaveImage(image_name, copy_img, 0);
                //cvResetImageROI(copy_img);

                cv::rectangle(*show_img, pt1, pt2, color, width, 8, 0);
                if (ext_output)
                    printf("\t(left_x: %4.0f   top_y: %4.0f   width: %4.0f   height: %4.0f)\n",
                    (float)left, (float)top, b.w*show_img->cols, b.h*show_img->rows);
                else
                    printf("\n");

                cv::rectangle(*show_img, pt_text_bg1, pt_text_bg2, color, width, 8, 0);
                cv::rectangle(*show_img, pt_text_bg1, pt_text_bg2, color, CV_FILLED, 8, 0);    // filled
                cv::Scalar black_color = CV_RGB(0, 0, 0);
                cv::putText(*show_img, labelstr, pt_text, cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, black_color, 2 * font_size, CV_AA);
                // cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SIMPLEX
            }
        }
        if (ext_output) {
            fflush(stdout);
        }
    }
    catch (...) {
        cerr << "OpenCV exception: draw_detections_cv_v3() \n";
    }

    // Reference
    std::vector<ObjectDetection> detections;

    for (size_t i = 0; i < dets[0].size(); i++)
    {
        ObjectDetection detection;

        image.cols;
        image.rows;

        detection.id = dets[i].track_id;
        detection.center_x = img.cols * dets[i].bbox.x;
        detection.center_y = img.rows * dets[i].bbox.y;
        detection.width_half = img.cols * dets[i].bbox.w / 2.0;
        detection.height_half = img.rows * static_cast<int>((b - t) / 2.f);
        detections.push_back(detection);
    }


    return detections;
}