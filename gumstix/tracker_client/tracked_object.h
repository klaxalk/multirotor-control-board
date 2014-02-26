/*
 * File name: tracked_object.h
 * Date:      2012/01/07 14:04
 * Author:    Jan Faigl
 */

#ifndef __TRACKED_OBJECT_H__
#define __TRACKED_OBJECT_H__

namespace imr {

   struct STrackedObject {
      bool valid; //true if the blob info is valid
      float x;
      float y;
      float z;
      float pitch;
      float roll;
      float yaw;
      float pixel_ratio; //ratio of the expected and detected pixels of the 
      float bw_ratio; //ratio of the detected color (grays)
   };

} //end namespace imr;


#endif

/* end of tracked_object.h */
