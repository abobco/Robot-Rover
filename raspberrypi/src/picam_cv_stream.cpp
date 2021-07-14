/*
    Basic JPEG image streaming protocol:

        1. send image size in bytes
        2. send image as JPEG file (lossy compression)

    - just like in picam_stream.cpp, network speed is bottleneck
    - can improve image throughput by lowering jpeg quality (80% has few skips)

    Possible improvements:
        - only send changed pixels (may be costly on the pi, probably need gpu
   access)
        - use a better protocol for video streaming (mpeg, rtsp)
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

static const int JPEG_QUALITY = 100; // default(95) 0-100

int sock;

int main(int argc, char **argv) {
  raspicam::RaspiCam_Cv Camera;
  cv::Mat image;

  // set camera params
  Camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
  Camera.set(cv::CAP_PROP_FPS, 16);
  Camera.set(cv::CAP_PROP_FRAME_WIDTH, 320 * 2);
  Camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240 * 2);

  // Open camera
  if (!Camera.open()) {
    std::cerr << "Error opening the camera" << std::endl;
    return -1;
  }

  sock = connect_socket_blocking(PORT_RPI_CAM);

  while (true) {
    Camera.grab();
    Camera.retrieve(image);

    std::vector<uchar> buff; // buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = JPEG_QUALITY;
    cv::imencode(".jpg", image, buff, param);

    uint32_t size = buff.size();
    DUMP(size);
    send_buf(sock, (char *)&size, sizeof(size));

    send_buf(sock, (char *)&buff.front(), size);
  }

  std::cout << "Stop camera..." << std::endl;
  Camera.release();
}