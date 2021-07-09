#include "app_state.hpp"
namespace xn {

AppState &AppState::get() {
  static AppState instance;
  return instance;
}

} // namespace xn