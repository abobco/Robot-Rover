#include "xn_gpio.hpp"

namespace xn {

void PigDaemon::init() { pi = 0; }

void PigDaemon::stop() { pi = -1; }

int PigDaemon::i2cOpen(unsigned bus, unsigned addr) { return 0; }

int PigDaemon::i2cClose(unsigned handle) { return 0; }

char PigDaemon::i2cReadByte(int h) { return 0; }

int PigDaemon::i2cRead(int h, char *buf, int n) { return 0; }

int PigDaemon::i2cWrite(int h, char *buf, int n) { return 0; }

int PigDaemon::servo(unsigned gpio, unsigned pulsewidth) { return 0; }

int PigDaemon::serialOpen(char *ser_tty, unsigned baud) { return 0; }

int PigDaemon::serialClose(int h) { return 0; }

int PigDaemon::serialRead(int h, char *buf, unsigned count) { return 0; }

int PigDaemon::serialWrite(int h, char *buf, unsigned count) { return 0; }

int PigDaemon::serialDataAvailable(int h) { return 0; }

namespace pio {} // namespace pio
} // namespace xn