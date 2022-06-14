#include "Camera.hpp"

cv::VideoCapture inputVideo("/dev/video0");
void Cam::Cam_Init()
{
    cameraMatrix.at<double>(0, 0) = 1.1527e+03;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 618.6327;
    cameraMatrix.at<double>(1, 1) = 1.1527e+03;
    cameraMatrix.at<double>(1, 2) = 361.1508;

    distCoeffs.at<double>(0, 0) = 0.0741;
    distCoeffs.at<double>(1, 0) = -0.0685;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = 0;

    //inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,200);//width
    //inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,100);//height

    //inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);//宽度
    //inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//高度
    //inputVideo.set(cv::CAP_PROP_FPS, 30);//帧率 帧/秒
    //inputVideo.set(cv::CAP_PROP_BRIGHTNESS, 14);//亮度
    //inputVideo.set(cv::CAP_PROP_CONTRAST, 35);//对比度
    //inputVideo.set(cv::CAP_PROP_SATURATION, 128);//饱和度
    //inputVideo.set(cv::CAP_PROP_HUE, -13);//调 50
    //inputVideo.set(cv::CAP_PROP_EXPOSURE, -11);//曝光
    // int gamm = 100;
    // int expo = 1;

    //inputVideo.set(cv::CAP_PROP_GAMMA, gamm);

    //inputVideo.set(cv::CAP_PROP_EXPOSURE, -4);

    //inputVideo.set(cv::CAP_PROP_EXPOSURE, expo);
    //inputVideo.set(cv::CAP_PROP_AUTO_EXPOSURE,2.6);
    // double fps = inputVideo.get(cv::CAP_PROP_FPS);
    // std::cout<<fps<<std::endl;
    return;
}

bool Cam::getImage()
{
    if (!inputVideo.isOpened())
    {
        std::cout << "Could not open the input video: " << std::endl;
        return false;
    }
    else
    {
        //inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);//宽度
        //inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//高度
        inputVideo.set(cv::CAP_PROP_FPS, 25);//帧率 帧/秒
        inputVideo.set(cv::CAP_PROP_GAMMA, 0);
        inputVideo.set(cv::CAP_PROP_BRIGHTNESS, 100);//亮度
        inputVideo.set(cv::CAP_PROP_CONTRAST, 28);//对比度
        inputVideo.set(cv::CAP_PROP_SATURATION, 100);//饱和度
        inputVideo.set(cv::CAP_PROP_HUE, 100);//调 50
        inputVideo.set(cv::CAP_PROP_EXPOSURE, 50);//曝光
        inputVideo.read(InitImg);
        return true;
    }
}
void Cam::imgAdjust()
{
    cv::Mat map1, map2;
    cv::Size imageSize;
    imageSize = InitImg.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                            imageSize, CV_16SC2, map1, map2);
    // check if at end
    remap(InitImg, AdjustedImg, map1, map2, cv::INTER_LINEAR);
    return;
}