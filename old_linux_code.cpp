/*
    This app was originally written for Windows subsystem for Linux, then ported to windows.
    
    Here are the incompatible linux functions that were replaced 
*/

// $1 to anyone that can show me a windows equivalent for this function with similar accuracy.
// my windows replacement can be off by as much as 1 ms !!
void time_sleep(double seconds) {
    struct timespec ts, rem;

    if (seconds > 0.0) {
        ts.tv_sec = seconds;
        ts.tv_nsec = (seconds - (double)ts.tv_sec) * 1E9;

        while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem)) {
            /* copy remaining time to ts */
            ts.tv_sec = rem.tv_sec;
            ts.tv_nsec = rem.tv_nsec;
        }
    }
}

// linux server-side blocking accept
int accept_connection_blocking(unsigned port) {
  int sockfd;
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  char enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");
  listen(sockfd, 5);
  clilen = sizeof(cli_addr);

  printf("waiting for connection...\n");
  int newsock = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
  close(sockfd);

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
  return newsock;
}

