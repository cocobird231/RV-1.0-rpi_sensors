#include "header.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("webcam_params_node");
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
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}