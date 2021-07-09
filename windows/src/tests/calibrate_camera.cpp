#define PIO_VIRTUAL
#include "calibrate_camera.hpp"

int main() {
  camcalib::calibrate_camera("in_VID5.xml");
  return 0;
}