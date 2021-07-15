#include "xn_console.hpp"
namespace xn {

SshConsole::SshConsole(const json &settings,
                       const std::string &history_filepath)
    : settings(settings), history_filepath(history_filepath) {
  init();
}

SshConsole::SshConsole(const json &settings, ssh_session *sesh,
                       const std::string &history_filepath)
    : settings(settings), sesh(*sesh), history_filepath(history_filepath) {
  init();
}

SshConsole::~SshConsole() {
  ClearLog();
  for (int i = 0; i < History.Size; i++)
    free(History[i]);
}

void SshConsole::init() {
  ClearLog();
  memset(InputBuf, 0, sizeof(InputBuf));
  HistoryPos = -1;

  std::fstream history_file;
  history_file.open(history_filepath, std::ios::in);
  if (history_file.is_open()) {
    std::string tp;
    while (std::getline(history_file, tp)) {
      History.push_back(Strdup(tp.c_str()));
    }
    history_file.close();
  }

  // "CLASSIFY" is here to provide the test case where "C"+[tab] completes
  // to "CL" and display multiple matches.
  Commands.push_back("HELP");
  Commands.push_back("HISTORY");
  Commands.push_back("CLEAR");
  Commands.push_back("CLASSIFY");
  AutoScroll = true;
  ScrollToBottom = false;
  AddLog("ssh terminal started");
}

// Portable helpers
int SshConsole::Stricmp(const char *s1, const char *s2) {
  int d;
  while ((d = toupper(*s2) - toupper(*s1)) == 0 && *s1) {
    s1++;
    s2++;
  }
  return d;
}
int SshConsole::Strnicmp(const char *s1, const char *s2, int n) {
  int d = 0;
  while (n > 0 && (d = toupper(*s2) - toupper(*s1)) == 0 && *s1) {
    s1++;
    s2++;
    n--;
  }
  return d;
}

char *SshConsole::Strdup(const char *s) {
  IM_ASSERT(s);
  size_t len = strlen(s) + 1;
  void *buf = malloc(len);
  IM_ASSERT(buf);
  return (char *)memcpy(buf, (const void *)s, len);
}

void SshConsole::Strtrim(char *s) {
  char *str_end = s + strlen(s);
  while (str_end > s && str_end[-1] == ' ')
    str_end--;
  *str_end = 0;
}

void SshConsole::ClearLog() {
  for (int i = 0; i < Items.Size; i++)
    free(Items[i]);
  Items.clear();
}

void SshConsole::AddLog(const char *fmt, ...) IM_FMTARGS(2) {
  // FIXME-OPT
  char buf[1024];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, IM_ARRAYSIZE(buf), fmt, args);
  buf[IM_ARRAYSIZE(buf) - 1] = 0;
  va_end(args);
  Items.push_back(Strdup(buf));
}

void SshConsole::Draw(const char *title, bool *p_open, RobotController &robot,
                      GridGraph &navgraph) {
  if (AppState::get().gui_windows_need_update) {
    ImVec2 win_center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(ImVec2(win_center.x * 2, win_center.y * 2), 0,
                            ImVec2(1, 1));
  }
  if (!ImGui::Begin(title, p_open)) {
    ImGui::End();
    return;
  }

  // As a specific feature guaranteed by the library, after calling
  // Begin() the last Item represent the title bar. So e.g.
  // IsItemHovered() will return true when hovering the title bar. Here we
  // create a context menu only available from the title bar.
  if (ImGui::BeginPopupContextItem()) {
    if (ImGui::MenuItem("Close Console"))
      *p_open = false;
    ImGui::EndPopup();
  }
  ImGui::TextWrapped("Enter 'HELP' for help.");

  // TODO: display items starting from the bottom

  if (ImGui::SmallButton("ssh rapi")) {
    xn_printf("sup\n");
    threads.push_back(std::thread([&]() {
      int r = ssh_login(sesh, settings["ssh_host"], settings["ssh_user"],
                        settings["ssh_password"], istream);
      if (r < 0) {
        std::cout << "failed to connect\n";
        return;
      }

      AppState::get().conn_accepted = true;
      std::vector<std::thread> jobs;
      jobs.reserve(5);
      jobs.push_back(std::thread(interactive_shell_session, sesh, " ",
                                 std::ref(ostream), std::ref(istream),
                                 std::ref(AppState::get().conn_accepted)));

      std::thread connection_jobs[SOCKET_COUNT];
      for (unsigned i = 0; i < SOCKET_COUNT; i++) {
        if (i == SOCKET_ESP_DATA)
          continue;
        connection_jobs[i] = std::thread([&]() {
          closesocket(sockets[i]);
          sockets[i] = accept_connection_blocking(BASE_PORT + i);
          if (sockets[i] < 0)
            error("ERROR on accept");
        });
      }

      for (unsigned i = 0; i < SOCKET_COUNT; i++) {
        if (i == SOCKET_ESP_DATA)
          continue;
        connection_jobs[i].join();
      }

      jobs.push_back(
          std::thread(esp_log_thread, std::ref(sockets[SOCKET_ESP_LOG])));
      jobs.push_back(
          std::thread(cam_io_thread, std::ref(sockets[SOCKET_RPI_CAM]),
                      std::ref(robot.cam_pic),
                      std::ref(robot.cam_outframe))); // camera stream
      jobs.push_back(std::thread(cam_servo_ctl_thread, settings,
                                 std::ref(sockets[SOCKET_RPI_CAM]),
                                 std::ref(robot)));
      jobs.push_back(
          std::thread(yolo_thread, settings, std::ref(sockets[SOCKET_RPI_ARM]),
                      std::ref(robot.armInfo.target), std::ref(robot.cam_pic),
                      std::ref(robot.cam_outframe)));
      int i = 0;
      for (auto &j : jobs) {
        j.join();
        std::cout << "thread " << i++ << " joined\n";
      }

      std::cout << "all ssh threads joined\n";
      // values
    }));
  }
  ImGui::SameLine();

  if (ImGui::SmallButton("connect esp32")) {
    threads.push_back(std::thread([&]() {
      closesocket(sockets[SOCKET_ESP_DATA]);
      sockets[SOCKET_ESP_DATA] =
          accept_connection_blocking(BASE_PORT + SOCKET_ESP_DATA);
      robot.rover.esp32 = sockets[SOCKET_ESP_DATA];
      robot.rover.wheelDiameter = 200 * (float)settings["wheel_diameter"];
      robot.rover.wheelSeparation = 200 * (float)settings["wheel_separation"];
      robot.rover.motorCpr = settings["motor_cpr"];
      rover_ctl_thread(robot.rover, navgraph);
    }));
  }
  // ImGui::SameLine();
  if (ImGui::SmallButton("Add Debug Error")) {
    AddLog("[error] something went wrong");
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear")) {
    ClearLog();
  }
  ImGui::SameLine();
  bool copy_to_clipboard = ImGui::SmallButton("Copy");
  // static float t = 0.0f; if (ImGui::GetTime() - t > 0.02f) { t =
  // ImGui::GetTime(); AddLog("Spam %f", t); }

  ImGui::Separator();

  // Options menu
  if (ImGui::BeginPopup("Options")) {
    ImGui::Checkbox("Auto-scroll", &AutoScroll);
    ImGui::EndPopup();
  }

  // Options, Filter
  if (ImGui::Button("Options"))
    ImGui::OpenPopup("Options");
  ImGui::SameLine();
  Filter.Draw("Filter (\"incl,-excl\") (\"error\")", 180);
  ImGui::Separator();

  // Reserve enough left-over height for 1 separator + 1 input text
  const float footer_height_to_reserve =
      ImGui::GetStyle().ItemSpacing.y + ImGui::GetFrameHeightWithSpacing();
  ImGui::BeginChild("ScrollingRegion", ImVec2(0, -footer_height_to_reserve),
                    false, ImGuiWindowFlags_HorizontalScrollbar);
  if (ImGui::BeginPopupContextWindow()) {
    if (ImGui::Selectable("Clear"))
      ClearLog();
    ImGui::EndPopup();
  }

  // Display every line as a separate entry so we can change their color
  // or add custom widgets. If you only want raw text you can use
  // ImGui::TextUnformatted(log.begin(), log.end()); NB- if you have
  // thousands of entries this approach may be too inefficient and may
  // require user-side clipping to only process visible items. The clipper
  // will automatically measure the height of your first item and then
  // "seek" to display only items in the visible area. To use the clipper
  // we can replace your standard loop:
  //      for (int i = 0; i < Items.Size; i++)
  //   With:
  //      ImGuiListClipper clipper;
  //      clipper.Begin(Items.Size);
  //      while (clipper.Step())
  //         for (int i = clipper.DisplayStart; i < clipper.DisplayEnd;
  //         i++)
  // - That your items are evenly spaced (same height)
  // - That you have cheap random access to your elements (you can access
  // them given their index,
  //   without processing all the ones before)
  // You cannot this code as-is if a filter is active because it breaks
  // the 'cheap random-access' property. We would need random-access on
  // the post-filtered list. A typical application wanting coarse clipping
  // and filtering may want to pre-compute an array of indices or offsets
  // of items that passed the filtering test, recomputing this array when
  // user changes the filter, and appending newly elements as they are
  // inserted. This is left as a task to the user until we can manage to
  // improve this example code! If your items are of variable height:
  // - Split them into same height items would be simpler and facilitate
  // random-seeking into your list.
  // - Consider using manual call to IsRectVisible() and skipping
  // extraneous decoration from your items.
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,
                      ImVec2(4, 1)); // Tighten spacing
  if (copy_to_clipboard)
    ImGui::LogToClipboard();
  for (int i = 0; i < Items.Size; i++) {
    const char *item = Items[i];
    if (!Filter.PassFilter(item))
      continue;

    // Normally you would store more information in your item than just a
    // string. (e.g. make Items[] an array of structure, store color/type
    // etc.)
    ImVec4 color;
    bool has_color = false;
    if (strstr(item, "[error]")) {
      color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
      has_color = true;
    } else if (strncmp(item, "# ", 2) == 0) {
      color = ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
      has_color = true;
    }
    if (has_color)
      ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextWrapped(item);
    if (has_color)
      ImGui::PopStyleColor();
  }
  if (copy_to_clipboard)
    ImGui::LogFinish();

  if (ScrollToBottom ||
      (AutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()))
    ImGui::SetScrollHereY(1.0f);
  ScrollToBottom = false;

  ImGui::PopStyleVar();
  ImGui::EndChild();
  ImGui::Separator();

  // Command-line
  bool reclaim_focus = false;
  ImGuiInputTextFlags input_text_flags =
      ImGuiInputTextFlags_EnterReturnsTrue |
      ImGuiInputTextFlags_CallbackCompletion |
      ImGuiInputTextFlags_CallbackHistory;
  if (ImGui::InputText("Input", InputBuf, IM_ARRAYSIZE(InputBuf),
                       input_text_flags, &TextEditCallbackStub, (void *)this)) {
    char *s = InputBuf;
    Strtrim(s);
    if (s[0])
      ExecCommand(s);
    strcpy(s, "");
    reclaim_focus = true;
  }

  // Auto-focus on window apparition
  ImGui::SetItemDefaultFocus();
  if (reclaim_focus)
    ImGui::SetKeyboardFocusHere(-1); // Auto focus previous widget

  ImGui::End();
}

} // namespace xn