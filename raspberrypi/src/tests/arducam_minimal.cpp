#include "XNlib/xn_gpio.hpp"
#include <fstream>

using namespace xn;
int pio::PigDaemon::pi = 0;
int main() {
    pio::Arducam cam;
    cam.setQuality(4);
    cam.capture();
    std::ofstream wf("image.jpeg", std::ios::out | std::ios::binary);
    wf.write(cam.frame_buffer, cam.frame_size);
    wf.close();
    return 0;
}