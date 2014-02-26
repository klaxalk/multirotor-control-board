/*
 * File name: tracker_client.h
 * Date:      2012/01/07 09:22
 * Author:    Jan Faigl
 */

#ifndef __TRACKER_CLIENT_H__
#define __TRACKER_CLIENT_H__

#include <string>

#include <netinet/in.h>

namespace imr {

   class CTrackerClient {
      const std::string HOST;
      const int PORT;
      const std::string MYIP;
      const int MYPORT;
      const unsigned int CONNECT_TIMEOUT;
      public:
      const int MAXBUFSIZE;
      private:
      int sd;
      struct sockaddr_in serv_addr;
      struct sockaddr_in addr;

      public:
      CTrackerClient(const std::string& host, const int port, const std::string& myip, int myport);
      ~CTrackerClient();

      bool isConnected(void) const { return sd != -1; }
      bool connect(void);
      void disconnect(void);

      int send(const std::string& cmd, const std::string& opts);
      int receive(char* buf, const int size, bool nonblock = false);
   };

} //end namespace imr


#endif

/* end of tracker_client.h */
