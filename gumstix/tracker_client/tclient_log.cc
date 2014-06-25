/*
 * File name: tclient_log.cc
 * Date:      2012/06/07 17:04
 * Author:    Jan Faigl
 */

#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>

#include <sys/socket.h>
#include <sys/time.h>
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
static struct timeval tvstart; 

/// ----------------------------------------------------------------------------
void terminate_handler(int sig) {
   if (client) {
      std::cerr << "Terminate client!" << std::endl;
      quit = true;
   }
}

/// ----------------------------------------------------------------------------
void print_object(unsigned long id, unsigned time, const imr::STrackedObject& obj) {
   std::cout << "Object id: " << id << " time: " << time 
      << " valid: " << obj.valid 
      << " coords: " << obj.x << " " << obj.y << " " << obj.z
      << " rotations: " << obj.pitch << " " << obj.roll << " " << obj.yaw
      << " pixel_ratio: " << obj.pixel_ratio << " bw_ratio: " << obj.bw_ratio
      << std::endl;
}

/// ----------------------------------------------------------------------------
void print_object_log(std::ofstream& out,  unsigned long id, unsigned time, const imr::STrackedObject& obj) {
   static struct timeval tv;
   gettimeofday(&tv, NULL);
   const double t =  tv.tv_sec - tvstart.tv_sec + tv.tv_usec / 1e6;
   out   
      << t << " " 
      << obj.valid << " "
      << obj.x << " "
      << obj.y << " "
      << obj.z << " "
      << std::endl;
}

/// - main function ------------------------------------------------------------
int main(int argc, char* argv[]) {
   int ret = -1;

   const std::string host = argc > 1 ? std::string(argv[1]) : "localhost";
   const int server_port  = argc > 2 ? atoi(argv[2])        : 9001;
   const std::string myip = argc > 3 ? std::string(argv[3]) : "127.0.0.1";
   const int client_port  = argc > 4 ? atoi(argv[4]) : 9005;
   const char* log_file = argc > 5 ? argv[5] : 0;
   const bool BLOCKING_READ = argc > 6 ? atoi(argv[6]) : false;
   gettimeofday(&tvstart, NULL);

   client = new imr::CTrackerClient(host, server_port, myip, client_port);
   signal(SIGINT, terminate_handler); /// register signal handler
   std::cerr << "INFO:"
      << " connect to the tracker " << host << ":" << server_port 
      << " using address " << myip  << ":" << client_port 
      << std::endl;
   // allocate buffer for the message
   const int SIZE = 2 * (sizeof(imr::STrackedObject) + 2 * sizeof(unsigned long));
   char buf[SIZE];
   unsigned long id;
   unsigned long time;
   imr::STrackedObject obj;

   std::ofstream out;

   if (log_file) {
      out.open(log_file);
   }

   if (BLOCKING_READ) {
      //An example of blocked receiving
      if (client->connect()) {
	 std::cerr << "INFO: Connected to the tracker server" << std::endl;
      } else {
	 std::cerr << "WARN: Connection to the tracker server fail!" << std::endl;
      }
      quit = !client->isConnected();
      while(!quit) {
	 std::cerr << "quit: " << quit << std::endl;
	 int l = client->receive(&buf[0], SIZE, false);
	 if (l > 0 and imr::unpack(&buf[0], SIZE, id, time, obj) > 0) {
	    if (log_file and out) {
	       print_object_log(out, id, time, obj);
	    } else {
	       print_object(id, time, obj);
	    }
	 } else if (l < 0 ) {
	    std::cerr << "WARN: Error in mesage receive" << std::endl;
	    usleep(100*1000);
	 }
      } //end receiving loop
   } else {
      // An example of non-blocked receive with reconnection
      const int MSG_WAIT = 10; //in ms
      const int MAX_CONNECT_FAILS = 10;
      const int MAX_RECEIVE_FAILS = 10;
      int state = 0;
      int numConnectRetry = 0;
      int numReceiveRetry = 0;

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
	       while(client->receive(&buf[0], SIZE, true) > 0) { //read all messages
		  if (imr::unpack(&buf[0], SIZE, id, time, obj) > 0) {
		     numReceiveRetry = 0;
		     if (log_file and out) {
			print_object_log(out, id, time, obj);
		     } else {
			print_object(id, time, obj);
		     }
		  }
	       }
	       if (numReceiveRetry < MAX_RECEIVE_FAILS) { //wait for a while
		  numReceiveRetry++;
		  usleep(MSG_WAIT * 1000);
	       } else { //
		  std::cerr << "Receive fails " << numReceiveRetry << " times, try to reconnect" << std::endl;
		  client->disconnect(); 
		  state = 0;
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

/* end of tclient_log.cc */
