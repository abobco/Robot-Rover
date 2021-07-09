#define _CRT_SECURE_NO_WARNINGS
#define PIO_VIRTUAL

#include "../XN_net.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <queue>
#include <thread>

// using namespace std;
using namespace cv;

SOCKET sock;

int main(int argc, const char **argv) {

  sock = xn::accept_connection_blocking(5002);

  while (waitKey(1) != 27) {
    uint32_t s;

    xn::read_int(sock, s, false);

    unsigned char *data = new unsigned char[s];

    xn::read_buf(sock, (char *)data, s, 10);
    // Mat rawData(1, s, CV_8UC1, (void *)data);
    Mat rawData(1, s, CV_8SC1, (void *)data);
    Mat decodedImage = imdecode(rawData, cv::IMREAD_UNCHANGED);
    // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
    imshow("img", decodedImage);

    delete[] data;
  }

  return 0;
}