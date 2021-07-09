#include "xn_log.hpp"

namespace xn {

StreamHook::StreamHook(std::ostream &inputStream) {
  this->inputStream = &inputStream;
  stream = std::stringstream();
  buf = inputStream.rdbuf(stream.rdbuf());
}

void StreamHook::dump(std::string &output) {
  output = stream.str();
  stream.str(std::string());
  buf = inputStream->rdbuf(buf);
  *inputStream << output;
  buf = inputStream->rdbuf(stream.rdbuf());
}

void LogWindow::AddLog(const char *fmt, ...) IM_PRINTFARGS(2) {
  va_list args;
  va_start(args, fmt);
  Buf.appendfv(fmt, args);
  va_end(args);
  ScrollToBottom = true;
}

void LogWindow::DrawTextWrapped() {
  ImGui::TextWrapped("%s", Buf.begin());
  if (ScrollToBottom)
    ImGui::SetScrollHere(1.0f);
  ScrollToBottom = false;
}

void LogWindow::Draw(const char *title, bool *p_opened) {
  ImGui::SetNextWindowBgAlpha(0.7);
  ImGui::Begin(title, p_opened);
  DrawTextWrapped();
  ImGui::End();
}

void xn_printf(const char *fmt, ...) IM_PRINTFARGS(2) {
  // get length of formatted string
  va_list args;
  va_start(args, fmt);
  size_t len = std::vsnprintf(NULL, 0, fmt, args);
  va_end(args);

  // print into string
  std::string str_formatted;
  str_formatted.reserve(len + 1);
  va_start(args, fmt);
  vsnprintf(&str_formatted[0], len + 1, fmt, args);
  va_end(args);

  std::cout << str_formatted;
}
} // namespace xn