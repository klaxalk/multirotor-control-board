/*
 * File name: serialization.h
 * Date:      2012/01/07 14:05
 * Author:    Jan Faigl
 */

#ifndef __SERIALIZATION_H__
#define __SERIALIZATION_H__

#include "tracked_object.h"

namespace imr {

   int pack(unsigned long id, unsigned long time, STrackedObject& obj, char* buf, size_t buflen);
   int unpack(char* buf, size_t buflen, unsigned long& id, unsigned long& time, STrackedObject& obj);

} //end namespace imr

#endif

/* end of serialization.h */
