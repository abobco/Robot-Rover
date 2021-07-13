#pragma once

#include "../app/app_threads_shared.hpp"
#include "xn_ssh.hpp"
#include <util/xn_json.hpp>

namespace xn {

class AppState;
struct RobotController;
struct GridGraph;

// simple console window with scrolling, filtering, completion and history.
struct SshConsole {
  char InputBuf[256];
  std::stringstream ostream, // consolestrea,
      istream;               // console_out_stream
  ssh_session sesh;
  ImVector<char *> Items;
  ImVector<const char *> Commands;
  ImVector<char *> History;
  int HistoryPos; // -1: new line, 0..History.Size-1 browsing history.
  ImGuiTextFilter Filter;
  bool AutoScroll;
  bool ScrollToBottom;
  std::vector<std::thread> threads;
  json settings;

  std::string history_filepath;

  static const unsigned BASE_PORT = 5002;
  enum SOCKET_INDEX {
    SOCKET_RPI_CAM,
    SOCKET_RPI_ARM,
    SOCKET_ESP_LOG,
    SOCKET_ESP_DATA,

    SOCKET_COUNT
  };

  SOCKET sockets[SOCKET_COUNT];

  // struct Sockets {
  //   SOCKET esp_data;
  //   SOCKET esp_log;
  //   SOCKET pi_arm;
  //   SOCKET pi_cam;
  // } sockets;

  // std::array<SOCKET, SOCKET_COUNT>

  SshConsole() {}

  SshConsole(const json &settings,
             const std::string &history_filepath = "history.txt");

  SshConsole(const json &settings, ssh_session *sesh,
             const std::string &history_filepath = "history.txt");

  ~SshConsole();

  void init();

  // Portable helpers
  static int Stricmp(const char *s1, const char *s2);
  static int Strnicmp(const char *s1, const char *s2, int n);
  static char *Strdup(const char *s);
  static void Strtrim(char *s);

  void ClearLog();
  void AddLog(const char *fmt, ...) IM_FMTARGS(2);

  void Draw(const char *title, bool *p_open, RobotController &robot,
            GridGraph &navgraph);

  void ExecCommand(const char *command_line) {
    AddLog("# %s\n", command_line);
    ostream << command_line;
    // Insert into history. First find match and delete it so it can be
    // pushed to the back. This isn't trying to be smart or optimal.
    HistoryPos = -1;
    for (int i = History.Size - 1; i >= 0; i--)
      if (Stricmp(History[i], command_line) == 0) {
        free(History[i]);
        History.erase(History.begin() + i);
        break;
      }
    History.push_back(Strdup(command_line));

    std::fstream history_file;
    history_file.open(history_filepath, std::ios_base::app);
    history_file << command_line << '\n';
    history_file.close();

    // Process command
    if (Stricmp(command_line, "CLEAR") == 0) {
      ClearLog();
    } else if (Stricmp(command_line, "HELP") == 0) {
      AddLog("Commands:");
      for (int i = 0; i < Commands.Size; i++)
        AddLog("- %s", Commands[i]);
    } else if (Stricmp(command_line, "HISTORY") == 0) {
      int first = History.Size - 10;
      for (int i = first > 0 ? first : 0; i < History.Size; i++)
        AddLog("%3d: %s\n", i, History[i]);
    }
    // else {
    //   // AddLog("Unknown command: '%s'\n", command_line);
    // }

    // On command input, we scroll to bottom even if AutoScroll==false
    ScrollToBottom = true;
  }

  // In C++11 you'd be better off using lambdas for this sort of forwarding
  // callbacks
  static int TextEditCallbackStub(ImGuiInputTextCallbackData *data) {
    SshConsole *console = (SshConsole *)data->UserData;
    return console->TextEditCallback(data);
  }

  int TextEditCallback(ImGuiInputTextCallbackData *data) {
    // AddLog("cursor: %d, selection: %d-%d", data->CursorPos,
    // data->SelectionStart, data->SelectionEnd);
    switch (data->EventFlag) {
    case ImGuiInputTextFlags_CallbackCompletion: {
      // Example of TEXT COMPLETION

      // Locate beginning of current word
      const char *word_end = data->Buf + data->CursorPos;
      const char *word_start = word_end;
      while (word_start > data->Buf) {
        const char c = word_start[-1];
        if (c == ' ' || c == '\t' || c == ',' || c == ';')
          break;
        word_start--;
      }

      // Build a list of candidates
      ImVector<const char *> candidates;
      for (int i = 0; i < Commands.Size; i++)
        if (Strnicmp(Commands[i], word_start, (int)(word_end - word_start)) ==
            0)
          candidates.push_back(Commands[i]);

      if (candidates.Size == 0) {
        // No match
        AddLog("No match for \"%.*s\"!\n", (int)(word_end - word_start),
               word_start);
      } else if (candidates.Size == 1) {
        // Single match. Delete the beginning of the word and replace it
        // entirely so we've got nice casing.
        data->DeleteChars((int)(word_start - data->Buf),
                          (int)(word_end - word_start));
        data->InsertChars(data->CursorPos, candidates[0]);
        data->InsertChars(data->CursorPos, " ");
      } else {
        // Multiple matches. Complete as much as we can..
        // So inputing "C"+Tab will complete to "CL" then display "CLEAR"
        // and "CLASSIFY" as matches.
        int match_len = (int)(word_end - word_start);
        for (;;) {
          int c = 0;
          bool all_candidates_matches = true;
          for (int i = 0; i < candidates.Size && all_candidates_matches; i++)
            if (i == 0)
              c = toupper(candidates[i][match_len]);
            else if (c == 0 || c != toupper(candidates[i][match_len]))
              all_candidates_matches = false;
          if (!all_candidates_matches)
            break;
          match_len++;
        }

        if (match_len > 0) {
          data->DeleteChars((int)(word_start - data->Buf),
                            (int)(word_end - word_start));
          data->InsertChars(data->CursorPos, candidates[0],
                            candidates[0] + match_len);
        }

        // List matches
        AddLog("Possible matches:\n");
        for (int i = 0; i < candidates.Size; i++)
          AddLog("- %s\n", candidates[i]);
      }

      break;
    }
    case ImGuiInputTextFlags_CallbackHistory: {
      // Example of HISTORY
      const int prev_history_pos = HistoryPos;
      if (data->EventKey == ImGuiKey_UpArrow) {
        if (HistoryPos == -1)
          HistoryPos = History.Size - 1;
        else if (HistoryPos > 0)
          HistoryPos--;
      } else if (data->EventKey == ImGuiKey_DownArrow) {
        if (HistoryPos != -1)
          if (++HistoryPos >= History.Size)
            HistoryPos = -1;
      }

      // A better implementation would preserve the data on the current
      // input line along with cursor position.
      if (prev_history_pos != HistoryPos) {
        const char *history_str = (HistoryPos >= 0) ? History[HistoryPos] : "";
        data->DeleteChars(0, data->BufTextLen);
        data->InsertChars(0, history_str);
      }
    }
    }
    return 0;
  }
};

// static void ShowSshConsole(bool *p_open) {
//   static SshConsole console;
//   console.Draw("Ssh: Console", p_open);
// }
} // namespace xn