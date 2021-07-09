/*
    Basic OpenCV image streaming protocol:

    - Frame size is hardcoded to 640x480 for both client and server
    - Image is sent directly as a cv::Mat
    - buggy and slow, used only as a reference for the other protocols

*/

#include <ctime>
#include <fstream>
#include <iostream>
#include <queue>
#include <thread>

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_net.hpp"

#include "raspicam_cv.h"
#include <opencv2/opencv.hpp>

using namespace xn;
int pio::PigDaemon::pi = 0;
int sock;

int main(int argc, char **argv) {
    raspicam::RaspiCam_Cv Camera;

    // image captue buffer
    cv::Mat img;
    img = cv::Mat::zeros(480, 640, CV_8UC3);
    if (!img.isContinuous()) {
        img = img.clone();
    }
    int imgSize = img.total() * img.elemSize();
    int bytes = 0;

    // set camera params
    Camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(cv::CAP_PROP_FPS, 60);
    Camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    Camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Open camera
    std::cout << "Opening Camera..." << std::endl;
    if (!Camera.open()) {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    sock = connect_socket_blocking(laptop_port);

    while (true) {
        Camera.grab();
        Camera.retrieve(img);
        send(sock, img.data, imgSize, 0);
    }

    std::cout << "Stop camera..." << std::endl;
    Camera.release();
}