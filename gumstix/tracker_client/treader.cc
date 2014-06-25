/*
 * File name: treader.cc
 * Date:      2012/01/06 07:27
 * Author:    Jan Faigl
 */

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

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
static inline long getTime(void) {
   struct timeval t;
   gettimeofday(&t, NULL);
   return t.tv_sec * 1000 + t.tv_usec / 1000;
}

/// ----------------------------------------------------------------------------
static inline float sqrt_positive(float v) {
   return v > 0.0 ? sqrt(v) : 0.0;
}

/// - main function ------------------------------------------------------------
int main(int argc, char* argv[]) {
   int ret = -1;

   const std::string host = argc > 1 ? std::string(argv[1]) : "localhost";
   const int server_port  = argc > 2 ? atoi(argv[2])        : 9001;
   const std::string myip = argc > 3 ? std::string(argv[3]) : "127.0.0.1";
   const int client_port  = argc > 4 ? atoi(argv[4]) : 9005;
   const bool BLOCKING_READ = argc > 5 ? atoi(argv[5]) : false;

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
	    print_object(id, time, obj);
	 } else if (l < 0 ) {
	    std::cerr << "WARN: Error in mesage receive" << std::endl;
	    usleep(100*1000);
	 }
      } //end receiving loop
   } else {
      // An example of non-blocked receive with reconnection
      const int MSG_WAIT = 10; //in ms
      const int MAX_CONNECT_FAILS = -1;
      //const int MAX_CONNECT_FAILS = 10;
      const int MAX_RECEIVE_FAILS = 10;
      int state = 0;
      int numConnectRetry = 0;
      int numReceiveRetry = 0;

      const long REPORT_PERIOD = 2 * 1000; //2 seconds
      const long REPORT_NUM_MSG = 100; //
      //const long REPORT_PERIOD = -1;
      unsigned long tStart;
      unsigned long tEnd;

      int nImages;
      int nValid;
      int numMsg = 0;
      imr::STrackedObject avg;
      imr::STrackedObject avg2;

      while(!quit) {
	 const int prev_state = state;
	 switch(state) {
	    case 0: //connecting to the server
	       if (client->connect()) {
		  numConnectRetry = 0;
		  numReceiveRetry = 0;
		  tStart = getTime();
		  nImages = nValid = 0;
		  avg.x = avg.y = avg.z = avg.roll = avg.pitch = avg.yaw = 0.0;
		  avg2.x = avg2.y = avg2.z = avg2.roll = avg2.pitch = avg2.yaw = 0.0;
		  std::cerr << "Connected!" << std::endl;
		  state = 1; //switch to receive the message
	       } else {
		  numConnectRetry++;
		  std::cerr << "Connection to server fails " << numConnectRetry << " times!" << std::endl;
	       }
	       if (MAX_CONNECT_FAILS >= 0 and numConnectRetry >= MAX_CONNECT_FAILS) { //decide what to do
		  std::cerr << "Connection to server fails too many times!" << std::endl;
		  quit = true; //exit the program
	       }
	       break;
	    case 1: //try to receive message
	       while(client->receive(&buf[0], SIZE, true) > 0) { //read all messages
		  if (imr::unpack(&buf[0], SIZE, id, time, obj) > 0) {
		     numReceiveRetry = 0;
		     numMsg++;
		     if (REPORT_PERIOD <= 0) {
			print_object(id, time, obj);
		     } else {
			tEnd = getTime();
		//	if ( (tEnd - tStart) >= REPORT_PERIOD) 
			if ( numMsg > REPORT_NUM_MSG) {
			   state = 2;
			} else {
			   nImages++;
			   if (obj.valid) {
			      nValid++;
			      avg.x += obj.x; avg.y += obj.y; avg.z += obj.z;
			      avg.roll += obj.roll; avg.pitch += obj.pitch; avg.yaw += obj.yaw;
			      avg2.x += obj.x*obj.x; avg2.y += obj.y*obj.y; avg2.z += obj.z*obj.z;
			      avg2.roll += obj.roll*obj.roll; avg2.pitch += obj.pitch*obj.pitch; avg2.yaw += obj.yaw*obj.yaw;
	//		      std::cout << " obj.x: " << obj.x << std::endl;
	//		      std::cout << " avg.x: " << avg.x << std::endl;
			   }
			}
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
	    case 2: //REPORT stats
	       {
		  const float n = nValid;
		  const unsigned long dt = tEnd - tStart;
		  avg.x /= n; avg.y /= n; avg.z /= n;
		  avg.roll /= n; avg.pitch /= n; avg.yaw /= n;
		  avg2.x /= n; avg2.y /= n; avg2.z /= n;
		  avg2.roll /= n; avg2.pitch /= n; avg2.yaw /= n;
		  imr::STrackedObject stdev;
		  stdev.x = sqrt_positive(avg2.x - avg.x*avg.x); stdev.y = sqrt_positive(avg2.y - avg.y*avg.y); stdev.z = sqrt_positive(avg2.z - avg.z*avg.z); 
		  stdev.roll = sqrt_positive(avg2.roll - avg.roll*avg.roll); stdev.pitch = sqrt_positive(avg2.pitch - avg.pitch*avg.pitch); stdev.yaw = sqrt_positive(avg2.yaw - avg.yaw*avg.yaw); 
		  int fps = (nImages > 0 and dt > 0) ? round(nImages / (dt / 1000.0)) : -1;
		  std::cerr 
		     << "Stats from " << nImages << " valid " << nValid 
		     << " period: " << REPORT_PERIOD << " ms real: " << dt << " ms " 
		     << " fps: " << fps  << " ratio: " << (nValid*1.0/nImages)
		     << std::endl
		     << "AVG: "
		     << " coords: " << avg.x << " " << avg.y << " " << avg.z
		     << " rotations: " << avg.pitch << " " << avg.roll << " " << avg.yaw
		     << std::endl
		     << "DEV: "
		     << " coords: " << stdev.x << " " << stdev.y << " " << stdev.z
		     << " rotations: " << stdev.pitch << " " << stdev.roll << " " << stdev.yaw
		     << std::endl;
		  tStart = getTime();
		  nImages = nValid = 0;
		  avg.x = avg.y = avg.z = avg.roll = avg.pitch = avg.yaw = 0.0;
		  avg2.x = avg2.y = avg2.z = avg2.roll = avg2.pitch = avg2.yaw = 0.0;
		  numMsg = 0;
		  state = 1;
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

/* end of treader.cc */
