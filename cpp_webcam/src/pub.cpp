#include "header.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("webcam_params_node");
    auto webcam_publisher = std::make_shared<RGBImagePublisher>(params);
    std::thread pubTh = std::thread(SpinNode, webcam_publisher, "webcam_publisher");
    
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
    pubTh.join();
    cap.release();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
