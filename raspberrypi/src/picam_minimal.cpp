/**
 */
#include "XNlib/xn_gpio.hpp"
#include "raspicam.h"
#include <ctime>
#include <fstream>
#include <iostream>

using namespace xn;
int pio::PigDaemon::pi = 0;

int main(int argc, char **argv) {

    raspicam::RaspiCam Camera; // Camera object
    // Open camera
    std::cout << "Opening Camera..." << std::endl;
    if (!Camera.open()) {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }
    // wait a while until camera stabilizes
    std::cout << "Sleeping for 3 secs" << std::endl;
    time_sleep(3);
    // capture
    Camera.grab();
    // allocate memory
    unsigned char *data = new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)];
    // extract the image in rgb format
    Camera.retrieve(data, raspicam::RASPICAM_FORMAT_RGB); // get camera image
    // save
    std::ofstream outFile("raspicam_image.ppm", std::ios::binary);
    outFile << "P6\n" << Camera.getWidth() << " " << Camera.getHeight() << " 255\n";
    outFile.write((char *)data, Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB));
    std::cout << "Image saved at raspicam_image.ppm" << std::endl;
    // free resrources
    delete[] data;
    return 0;
}