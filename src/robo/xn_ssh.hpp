#pragma once
#include "../util/xn_vec.hpp"
#include "xn_log.hpp"
#include <conio.h>
#include <errno.h>
#include <io.h>
#include <libssh/libssh.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(WIN32) || defined(WIN64)
#define strcasecmp _stricmp
#endif /* Def WIN32 or Def WIN64 */
namespace xn {
int verify_knownhost(ssh_session session);

std::string getpass(std::string msg);

int show_remote_processes(ssh_session session);

int ssh_exec_shell_cmd(ssh_session sesh, const std::string &cmd);

int ssh_login(ssh_session &sesh, std::string host, std::string username,
              std::string password, std::stringstream &ostream);

int interactive_shell_session(ssh_session session, const std::string start_cmd,
                              std::stringstream &input_stream,
                              std::stringstream &output_stream, bool &loopvar);

void ssh_quit(ssh_session s);

} // namespace xn