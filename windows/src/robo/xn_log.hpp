#pragma once

#include "imgui.h"
#include <iostream>
#include <sstream>
#include <stdio.h>

namespace xn {

struct StreamHook {
  std::stringstream stream;
  std::streambuf *buf;
  std::ostream *inputStream;

  StreamHook() {}

  StreamHook(std::ostream &inputStream);

  void dump(std::string &output);

  int bytesAvailable() { return (int)stream.tellp(); }
};

#define IM_PRINTFARGS(FMT)
struct LogWindow {
  ImGuiTextBuffer Buf;
  bool ScrollToBottom;

  void Clear() { Buf.clear(); }

  void AddLog(const char *fmt, ...) IM_PRINTFARGS(2);

  void AppendChars(const char *c_str) { Buf.append(c_str); }

  void DrawTextWrapped();

  void Draw(const char *title, bool *p_opened = NULL);
};

static LogWindow log_local;
static LogWindow log_rpi;
static LogWindow log_esp;

// printf to std::cout to capture logs easier
void xn_printf(const char *fmt, ...) IM_PRINTFARGS(2);
} // namespace xn