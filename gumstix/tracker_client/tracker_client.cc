/*
 * File name: tracker_client.cc
 * Date:      2012/01/07 09:22
 * Author:    Jan Faigl
 */

#include <cassert>
#include <sstream>
#include <cstring>

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "tracker_client.h"

using namespace imr;

/// - static method ------------------------------------------------------------

/// - constructor --------------------------------------------------------------
CTrackerClient::CTrackerClient(const std::string& host, const int port, const std::string& myip, int myport) :
   HOST(host), PORT(port), MYIP(myip), MYPORT(myport),
   CONNECT_TIMEOUT(1000),
   MAXBUFSIZE(1024)
{
   sd = -1;
}

/// - destructor ---------------------------------------------------------------
CTrackerClient::~CTrackerClient() {
   if (sd != -1) {
      disconnect();
   }
}

/// - public method ------------------------------------------------------------
bool CTrackerClient::connect(void) {
   bool ret = false;
   if (sd != -1) { disconnect(); }
   struct hostent *h = gethostbyname(HOST.c_str());
   assert(h);
   bzero((char*)&serv_addr, sizeof(serv_addr));
   serv_addr.sin_family = AF_INET; 
   bcopy((char *)h->h_addr, (char *)&serv_addr.sin_addr.s_addr, h->h_length);
   serv_addr.sin_port = htons(PORT);

   addr.sin_family = AF_INET;
   addr.sin_port = htons(MYPORT < 0 ? 0 : MYPORT);
   if (MYIP.empty()) {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
   } else {
      inet_pton(AF_INET, MYIP.c_str(), &addr.sin_addr);
   }
   sd = socket(AF_INET, SOCK_DGRAM, 0);
   assert(sd != -1);
   assert(bind(sd, (struct sockaddr*)&addr, sizeof(addr)) >= 0);

   // send connect
   std::stringstream ss;
   ss << "connect " << " " << inet_ntoa(addr.sin_addr) << " " << ntohs(addr.sin_port);
   const char* str = ss.str().c_str();
   assert(sendto(sd, str, strlen(str) + 1, 0, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) != -1);

   /// wait for receive
   bool end = false;
   char* buf = new char[MAXBUFSIZE];
   unsigned long timed = 0;
   socklen_t addr_len;
   while(!end) {
      int l = recvfrom(sd, &buf[0], MAXBUFSIZE, MSG_DONTWAIT, (struct sockaddr*)&serv_addr, &addr_len);
      if (l > 0 and buf[l-1] == '\0') {
	 std::string s(&buf[0]);
	 if (s == "accept") {
	    end = ret = true;
	 } else {
	    end = true;
	 }
      } else if (timed < CONNECT_TIMEOUT) {
	 usleep(100 * 1000); //sleep for 100 ms
	 timed += 100;
      } else {
	 end = true;
      }
   }
   delete[] buf;
   if (sd != -1 and !ret) {
      close(sd);
      sd = -1;
   }
   return ret;
}

/// - public method ------------------------------------------------------------
void CTrackerClient::disconnect(void) {
   if (sd != -1) {
      const char* s = std::string("disconnect").c_str();
      sendto(sd, s, strlen(s) + 1, 0, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
      close(sd);
      sd = -1;
   }
}

/// - public method ------------------------------------------------------------
int CTrackerClient::send(const std::string& cmd, const std::string& opts) {
   std::stringstream ss;
   ss 
      << cmd 
      << " " << inet_ntoa(addr.sin_addr) << " " << ntohs(addr.sin_port)
      << " " << opts;

   const char* str = ss.str().c_str();
   return sendto(sd, str, strlen(str) + 1, 0, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
}

/// - public method ------------------------------------------------------------
int CTrackerClient::receive(char* buf, const int size, bool nonblock) {
   int r = -1;
   if (sd != -1) {
      socklen_t len;
      r = recvfrom(sd, &buf[0], size , nonblock ? MSG_DONTWAIT : 0, (struct sockaddr*)&serv_addr, &len);
   }
   return r;
}

/// - private method -----------------------------------------------------------


/* end of tracker_client.cc */
