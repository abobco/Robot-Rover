#define _CRT_SECURE_NO_WARNINGS
#define PIO_VIRTUAL

#include "../XN_net.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <queue>
#include <thread>

using namespace xn;

SOCKET sock;

int main(int argc, const char **argv) {

  cv::Mat img;
  img = cv::Mat::zeros(480, 640, CV_8UC3);
  int imgSize = img.total() * img.elemSize();
  uchar *iptr = img.data;
  int bytes = 0;
  if (!img.isContinuous()) {
    img = img.clone();
  }

  sock = xn::accept_connection_blocking(5002);

  while (cv::waitKey(1) != 27) {
    uint32_t s;

    xn::read_int(sock, s, false);

    unsigned char *data = new unsigned char[s];

    xn::read_buf(sock, (char *)iptr, imgSize, 10);
    // Mat rawData(1, s, CV_8UC1, (void *)data);
    // Mat rawData(1, s, CV_8SC1, (void *)data);
    // Mat decodedImage = imdecode(rawData, cv::IMREAD_UNCHANGED);
    // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
    cv::imshow("img", img);
  }

  return 0;
}