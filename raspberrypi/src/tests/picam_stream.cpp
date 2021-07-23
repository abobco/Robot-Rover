/*
    Basic (and slow) protocol to stream the camera feed over a TCP socket

        1. send image size in bytes
        2. send image as a ppm file (ASCII image format)

    - current bottleneck is the network speed
    - server receives images at  ~2.5 fps
    - camera can produce 640x480 images at ~100 fps
    - faster framerate => slower shutter speed => darker images
*/

#include "XNlib/xn_gpio.hpp"
#include "XNlib/xn_net.hpp"

#include "raspicam.h"
#include <ctime>
#include <fstream>
#include <iostream>
#include <queue>
#include <thread>

using namespace xn;
int pio::PigDaemon::pi = 0;

int sock;

struct ImageData {
  uint32_t size;
  unsigned char *buffer;
};

std::queue<ImageData> frame_queue;

void frameUploadThread() {
  while (true) {
    auto t1 = pio::get_time();
    ImageData frame = {0};
    while (!frame_queue.empty()) {
      ImageData data = frame_queue.front();
      frame_queue.pop();
      if (frame_queue.empty()) {
        frame = data;
        break;
      }
      delete[] data.buffer;
    }
    if (frame.size > 0) {
      send(sock, (char *)&frame.size, sizeof(frame.size), 0);
      send(sock, (char *)frame.buffer, frame.size, 0);
    }
    delete[] frame.buffer;
    std::cout << "time elapsed: " << pio::time_diff_seconds(t1) << '\n';
  }
}

int main(int argc, char **argv) {
  raspicam::RaspiCam Camera; // Camera object
  Camera.setWidth(640);
  Camera.setHeight(480);
  Camera.setFrameRate(30);
  // Open camera
  std::cout << "Opening Camera..." << std::endl;
  if (!Camera.open()) {
    std::cerr << "Error opening camera" << std::endl;
    return -1;
  }

  sock = connect_socket_blocking(PORT_RPI_CAM);

  std::thread tid(frameUploadThread);

  while (true) {
    // auto t1 = pio::get_time();
    // capture
    Camera.grab();
    // allocate memory
    std::string header("P6\n" + std::to_string(Camera.getWidth()) + " " +
                       std::to_string(Camera.getHeight()) + " 255\n");
    const uint32_t img_size =
        Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB) + header.size();
    unsigned char *data = new unsigned char[img_size];
    header.copy((char *)data, header.size());
    // extract the image in rgb format
    Camera.retrieve(data + header.size()); // get camera image

    frame_queue.push({img_size, data});

    // std::cout << "time elapsed: " << pio::time_diff_seconds(t1) << '\n';
  }

  return 0;
}