/*
 * File name: tclient.cc
 * Date:      2012/07/03 11:28
 * Author:    Jan Faigl
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cstdlib>

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

#include <unistd.h>
#include <signal.h>

#include "tracked_object.h"
#include "serialization.h"
#include "tracker_client.h"


/// ----------------------------------------------------------------------------
imr::CTrackerClient* client = 0;
bool quit = false;

/// ----------------------------------------------------------------------------
void terminate_handler(int sig) {
   if (client) {
      std::cerr << "Terminate client!" << std::endl;
      quit = true;
   }
}

/// - main function ------------------------------------------------------------
int main(int argc, char* argv[]) {
   int ret = -1;

   const std::string host = argc > 1 ? std::string(argv[1]) : "localhost";
   const int server_port  = argc > 2 ? atoi(argv[2])        : 9001;
   const std::string myip = argc > 3 ? std::string(argv[3]) : "127.0.0.1";
   const int client_port  = argc > 4 ? atoi(argv[4]) : 9005;
   const bool BLOCKING_READ = argc > 5 ? atoi(argv[5]) : false;
   //const std::string EXT = "raw";
   const std::string EXT = "jpg";

   client = new imr::CTrackerClient(host, server_port, myip, client_port);
   signal(SIGINT, terminate_handler); /// register signal handler
   std::cerr << "INFO:"
      << " connect to the image server " << host << ":" << server_port 
      << " using address " << myip  << ":" << client_port 
      << std::endl;
   // allocate buffer for the message
   const int SIZE = client->MAXBUFSIZE;
   char buf[SIZE];
   unsigned long id;
   unsigned long time;

   if (BLOCKING_READ) {
      //An example of blocked receiving
      if (client->connect()) {
	 std::cerr << "INFO: Connected to the image server" << std::endl;
      } else {
	 std::cerr << "WARN: Connection to the image server fail!" << std::endl;
      }
      quit = !client->isConnected();
      int c;
      while(!quit) {
	 std::stringstream opts;
	 opts << "store_local_" << EXT << " " << std::setw(6) << std::setfill('0') << c << "." << EXT;
	 std::cerr << "send : " << opts.str() << std::endl;
	 if (client->send("capture", opts.str()) > 0) {
	    int l = client->receive(&buf[0], SIZE, false);
	    if (l > 0 and buf[l-1] == '\0') {
	       std::cerr << "Server response '" << buf << "'" << std::endl;
	       std::string s(buf);
	       if (s == "request accepted") {
		  c++;
	       }
	    } else if (l < 0 ) {
	       std::cerr << "WARN: Error in mesage receive" << std::endl;
	    }
	 } //end receiving loop
	 usleep(500*1000); //wait for 500 ms
      }
   } else {
      // An example of non-blocked receive with reconnection
      const int MSG_WAIT = 10; //in ms
      const int MAX_CONNECT_FAILS = 10;
      const int MAX_RECEIVE_FAILS = 10;
      int state = 0;
      int numConnectRetry = 0;
      int numReceiveRetry = 0;
      int c = 0; //counter

      while(!quit) {
	 const int prev_state = state;
	 switch(state) {
	    case 0: //connecting to the server
	       if (client->connect()) {
		  numConnectRetry = 0;
		  numReceiveRetry = 0;
		  state = 1; //switch to receive the message
	       } else {
		  numConnectRetry++;
		  std::cerr << "Connection to server fails " << numConnectRetry << " times!" << std::endl;
	       }
	       if (numConnectRetry >= MAX_CONNECT_FAILS) { //decide what to do
		  std::cerr << "Connection to server fails too many times!" << std::endl;
		  quit = true; //exit the program
	       }
	       break;
	    case 1: //try to receive message
	       {
		  std::stringstream opts;
		  opts << "store_local_" << EXT << " " << std::setw(6) << std::setfill('0') << c << "." << EXT;
		  std::cerr << "send : " << opts.str() << std::endl;
		  if (client->send("capture", opts.str()) > 0) {
		     int l = client->receive(&buf[0], SIZE, false);
		     if (l > 0 and buf[l-1] == '\0') {
			std::cerr << "Server response '" << buf << "'" << std::endl;
			std::string s(buf);
			if (s == "request accepted") {
			   c++;
			}
		     } else if (l < 0 ) {
			std::cerr << "WARN: Error in mesage receive" << std::endl;
			numConnectRetry++;
		     }
		  } else {
		     numConnectRetry++;
		  } 
		  if (numReceiveRetry < MAX_RECEIVE_FAILS) { //wait for a while
		     numReceiveRetry++;
		     usleep(MSG_WAIT * 1000);
		  } else { //
		     std::cerr << "Receive fails " << numReceiveRetry << " times, try to reconnect" << std::endl;
		     client->disconnect(); 
		     state = 0;
		  }
		  usleep(500*1000); //wait for 500 ms
	       }
	       break;
	 } //end switch
	 if (prev_state != state) {
	    std::cerr << "Change state " << prev_state << "->" << state << std::endl;
	 }
      }
      client->disconnect();
   }
   return ret;
}

/* end of tclient.cc */
