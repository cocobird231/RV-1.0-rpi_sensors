#include "header.h"

int main_send(std::shared_ptr<Params> params)
{
    auto webcam_publisher = std::make_shared<RGBImagePublisher>(params);
    
    cv::VideoCapture cap;
    cap.open(params->camera_cap_id);
    if (!cap.isOpened())
    {
        std::cerr << "Unable to open camera\n";
        return EXIT_FAILURE;
    }
    printf("Camera detached.\n");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, params->camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, params->camera_height);
    cap.set(cv::CAP_PROP_FPS, params->camera_fps);

    cv::Size pubImgSize(params->topic_Webcam_width, params->topic_Webcam_height);
    cv::Mat frame;

    std::vector<int> encodeParam;
    encodeParam.push_back(cv::IMWRITE_JPEG_QUALITY);
    encodeParam.push_back(70);
    std::vector<uchar> pubImgVec;

    int ret = cap.read(frame);
    if (!ret || frame.rows <= 0 || frame.cols <= 0)
    {
        std::cerr << "Unable to retrieve image\n";
        return EXIT_FAILURE;
    }
    if (frame.size() != pubImgSize)
        printf("The image will be resized into %dx%d. Current image size: %dx%d\n", 
                    pubImgSize.width, pubImgSize.height, frame.cols, frame.rows);

    while (1)
    {
        ret = cap.read(frame);
        if (!ret || frame.rows <= 0 || frame.cols <= 0)
        {
            std::cerr << "Unable to retrieve image\n";
            return EXIT_FAILURE;
        }
        if (!params->camera_use_color)
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        if (frame.size() != pubImgSize)
            cv::resize(frame, frame, pubImgSize);
        cv::imencode(".jpg", frame, pubImgVec, encodeParam);
        webcam_publisher->pubImage(pubImgVec, pubImgSize);
        cv::waitKey(1);
    }
    cap.release();
    return EXIT_SUCCESS;
}

int main_recv(std::shared_ptr<Params> params)
{
    cv::Mat initMat = cv::Mat(360, 640, CV_8UC3, cv::Scalar(100));
    auto webcam_subscriber = std::make_shared<RGBImageSubscriber>(params, initMat);
    std::thread subTh = std::thread(SpinNode, webcam_subscriber, "webcam_subscriberTh");

    cv::Mat src, dst;
    src = initMat.clone();
    dst = initMat.clone();
    WorkingRate wr(1000);
    wr.start();
    while (1)
    {
        if (webcam_subscriber->getRecvMat_clone(src))
        {
            dst = src.clone();
            wr.addOneCnt();
        }
        cv::Mat out = dst.clone();
        cv::resize(out, out, cv::Size(640, 360));
        cv::putText(out, "fps:" + std::to_string((int)wr.getRate()), cv::Point(10, out.rows - 10), 1, 4, cv::Scalar(0, 255, 255), 2);
        cv::imshow("dst", out);
        cv::waitKey(1);
    }
    subTh.join();
    return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("webcam_params_node");
    if (params->operationMode == "send")
        main_send(params);
    else if (params->operationMode == "recv")
        main_recv(params);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}