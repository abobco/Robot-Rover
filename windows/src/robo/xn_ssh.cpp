
// clang-format off
#include "winsock2.h"
#include "xn_ssh.hpp"
// clang-format on

namespace xn {

int verify_knownhost(ssh_session session) {
  enum ssh_known_hosts_e state;
  unsigned char *hash = NULL;
  ssh_key srv_pubkey = NULL;
  size_t hlen;
  char buf[10];
  char *hexa;
  char *p;
  int cmp;
  int rc;
  rc = ssh_get_server_publickey(session, &srv_pubkey);
  if (rc < 0) {
    return -1;
  }
  rc =
      ssh_get_publickey_hash(srv_pubkey, SSH_PUBLICKEY_HASH_SHA1, &hash, &hlen);
  ssh_key_free(srv_pubkey);
  if (rc < 0) {
    return -1;
  }
  state = ssh_session_is_known_server(session);
  switch (state) {
  case SSH_KNOWN_HOSTS_OK:
    /* OK */
    break;
  case SSH_KNOWN_HOSTS_CHANGED:
    fprintf(stderr, "Host key for server changed: it is now:\n");
    ssh_print_hexa("Public key hash", hash, hlen);
    fprintf(stderr, "For security reasons, connection will be stopped\n");
    ssh_clean_pubkey_hash(&hash);
    return -1;
  case SSH_KNOWN_HOSTS_OTHER:
    fprintf(stderr, "The host key for this server was not found but an other"
                    "type of key exists.\n");
    fprintf(stderr,
            "An attacker might change the default server key to"
            "confuse your client into thinking the key does not exist\n");
    ssh_clean_pubkey_hash(&hash);
    return -1;
  case SSH_KNOWN_HOSTS_NOT_FOUND:
    fprintf(stderr, "Could not find known host file.\n");
    fprintf(stderr, "If you accept the host key here, the file will be"
                    "automatically created.\n");
    /* FALL THROUGH to SSH_SERVER_NOT_KNOWN behavior */
  case SSH_KNOWN_HOSTS_UNKNOWN:
    hexa = ssh_get_hexa(hash, hlen);
    fprintf(stderr, "The server is unknown. Do you trust the host key?\n");
    fprintf(stderr, "Public key hash: %s\n", hexa);
    ssh_string_free_char(hexa);
    ssh_clean_pubkey_hash(&hash);
    p = fgets(buf, sizeof(buf), stdin);
    if (p == NULL) {
      return -1;
    }
    cmp = strcasecmp(buf, "yes");
    if (cmp != 0) {
      return -1;
    }
    rc = ssh_session_update_known_hosts(session);
    if (rc < 0) {
      fprintf(stderr, "Error %s\n", strerror(errno));
      return -1;
    }
    break;
  case SSH_KNOWN_HOSTS_ERROR:
    fprintf(stderr, "Error %s", ssh_get_error(session));
    ssh_clean_pubkey_hash(&hash);
    return -1;
  }
  ssh_clean_pubkey_hash(&hash);
  return 0;
}

std::string getpass(std::string msg) {
  std::string p;
  std::cout << msg;
  std::cin >> p;
  return p;
}

int show_remote_processes(ssh_session session) {
  ssh_channel channel;
  int rc;
  char buffer[256];
  int nbytes;
  channel = ssh_channel_new(session);
  if (channel == NULL)
    return SSH_ERROR;
  rc = ssh_channel_open_session(channel);
  if (rc != SSH_OK) {
    ssh_channel_free(channel);
    return rc;
  }
  rc = ssh_channel_request_exec(channel, "ps aux");
  if (rc != SSH_OK) {
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    return rc;
  }
  nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
  while (nbytes > 0) {
    if (_write(1, buffer, nbytes) != (unsigned int)nbytes) {
      ssh_channel_close(channel);
      ssh_channel_free(channel);
      return SSH_ERROR;
    }
    nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
  }
  if (nbytes < 0) {
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    return SSH_ERROR;
  }
  ssh_channel_send_eof(channel);
  ssh_channel_close(channel);
  ssh_channel_free(channel);
  return SSH_OK;
}

int ssh_exec_shell_cmd(ssh_session sesh, const std::string &cmd) {
  // create ssh channel
  ssh_channel channel;
  int rc;
  channel = ssh_channel_new(sesh);
  if (channel == NULL)
    return SSH_ERROR;
  rc = ssh_channel_open_session(channel);
  if (rc != SSH_OK) {
    ssh_channel_free(channel);
    return rc;
  }

  rc = ssh_channel_request_pty(channel);
  if (rc != SSH_OK)
    return rc;
  rc = ssh_channel_change_pty_size(channel, 80, 24);
  if (rc != SSH_OK)
    return rc;
  rc = ssh_channel_request_shell(channel);
  if (rc != SSH_OK)
    return rc;

  // start remote cmd
  std::cout << "ssh -> " << cmd << std::endl;
  char buffer[2048];
  int nbytes, nwritten;
  std::string cmd_full = cmd + "\n";
  nwritten = ssh_channel_write(channel, cmd_full.c_str(), cmd_full.size());
  while (ssh_channel_is_open(channel) && !ssh_channel_is_eof(channel)) {
    nbytes = ssh_channel_read_nonblocking(channel, buffer, sizeof(buffer), 0);
    if (nbytes < 0)
      return SSH_ERROR;
    if (nbytes > 0) {
      std::string strbuf;
      strbuf.assign(buffer, nbytes);
      std::cout << strbuf;
      log_rpi.AddLog("%s", strbuf.c_str());
    }
    time_sleep(0.05);
  }

  // close ssh channel
  ssh_channel_send_eof(channel);
  ssh_channel_close(channel);
  ssh_channel_free(channel);
  return SSH_OK;
}

int ssh_login(ssh_session &sesh, std::string host, std::string username,
              std::string password, std::stringstream &ostream) {
  ostream << "ssh " << username << "@" << host << "...\n";
  int rc;
  // Open session and set options
  sesh = ssh_new();
  if (sesh == NULL)
    return -1;
  ssh_options_set(sesh, SSH_OPTIONS_HOST, host.c_str());
  // Connect to server
  rc = ssh_connect(sesh);
  if (rc != SSH_OK) {
    // fprintf(stderr, "Error connecting to raspberrypi: %s\n",
    //         ssh_get_error(sesh));
    ostream << "[error] Error connecting to raspberrypi:" << ssh_get_error(sesh)
            << '\n';
    ssh_free(sesh);

    return -1;
    // exit(-1);
    // return sesh;
  }
  // Verify the server's identity
  rc = verify_knownhost(sesh);
  if (rc < 0) {
    ssh_disconnect(sesh);
    ssh_free(sesh);

    return -1;
    // exit(-1);
    // return sesh;
  }
  // Authenticate ourselves
  rc = ssh_userauth_password(sesh, username.c_str(), password.c_str());
  if (rc != SSH_AUTH_SUCCESS) {
    fprintf(stderr, "Error authenticating with password: %s\n",
            ssh_get_error(sesh));
    ssh_disconnect(sesh);
    ssh_free(sesh);
    return -1;
    // exit(-1);
    // return sesh;
  }
  xn_printf("connected\n");

  return 0;
}

int interactive_shell_session(ssh_session session, const std::string start_cmd,
                              std::stringstream &input_stream,
                              std::stringstream &output_stream, bool &loopvar) {
  ssh_channel channel;
  int rc;
  channel = ssh_channel_new(session);
  if (channel == NULL)
    return SSH_ERROR;
  rc = ssh_channel_open_session(channel);
  if (rc != SSH_OK) {
    ssh_channel_free(channel);
    return rc;
  }

  // rc = ssh_channel_request_pty(channel);
  // if (rc != SSH_OK)
  //   return rc;
  // rc = ssh_channel_change_pty_size(channel, 80, 24);
  // if (rc != SSH_OK)
  //   return rc;
  rc = ssh_channel_request_shell(channel);
  if (rc != SSH_OK)
    return rc;

  int nbytes, nwritten;
  if (start_cmd.size() > 0) {
    std::string cmd_full = start_cmd + "\n";
    nwritten = ssh_channel_write(channel, cmd_full.c_str(), cmd_full.size());
  }
  while (ssh_channel_is_open(channel) && !ssh_channel_is_eof(channel) &&
         loopvar) {

    char buffer[2048];
    nbytes = ssh_channel_read_nonblocking(channel, buffer, sizeof(buffer), 0);
    if (nbytes < 0) {
      return SSH_ERROR;
    }
    if (nbytes > 0) {
      nwritten = write(1, buffer, nbytes);
      output_stream.write(buffer, nbytes);
      if (nwritten != nbytes) {
        return SSH_ERROR;
      }
    }
    if (input_stream.tellp() == 0) {
      time_sleep(0.05);
      continue;
    }
    // nbytes = read(input_fd, buffer, sizeof(buffer));
    std::string contents;
    std::ostringstream os;
    contents = input_stream.str() + '\n';
    input_stream.str(std::string());
    // input_stream.str(std::string());
    // if (nbytes < 0)
    //   return SSH_ERROR;
    if (contents.size() > 0) {
      nwritten = ssh_channel_write(channel, contents.c_str(), contents.size());
      if (nwritten != contents.size())
        return SSH_ERROR;
    }
  }

  ssh_channel_close(channel);
  ssh_channel_send_eof(channel);
  ssh_channel_free(channel);
  return rc;
}

void ssh_quit(ssh_session s) {
  ssh_disconnect(s);
  ssh_free(s);
}

} // namespace xn