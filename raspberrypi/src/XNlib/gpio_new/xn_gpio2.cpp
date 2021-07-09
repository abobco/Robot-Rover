#include "xn_gpio2.hpp"

using namespace xn;
using namespace pio;

int PigDaemon::pi = 0;

TimePoint get_time() { return std::chrono::high_resolution_clock::now(); }

double time_diff_seconds(const TimePoint &t1, const TimePoint &t2) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
}

double time_diff_seconds(const TimePoint &t1) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(pio::get_time() - t1).count();
}

