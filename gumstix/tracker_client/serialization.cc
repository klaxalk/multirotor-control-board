/*
 * File name: serialization.cc
 * Date:      2012/01/07 14:08
 * Author:    Jan Faigl
 */

#include <rpc/types.h>
#include <rpc/xdr.h>

#include "serialization.h"

namespace imr {

   static int convert(unsigned long& id, unsigned long& time, STrackedObject& obj, char* buf, size_t buflen, int op);

   /// - public function ---------------------------------------------------------
   int pack(unsigned long id, unsigned long time, STrackedObject& obj, char* buf, size_t buflen) {
      return convert(id, time, obj, buf, buflen, XDR_ENCODE);
   }

   /// - public function ----------------------------------------------------------
   int unpack(char* buf, size_t buflen, unsigned long& id, unsigned long& time, STrackedObject& obj) {
      return convert(id, time, obj, buf, buflen, XDR_DECODE);
   }

   /// - private function ---------------------------------------------------------
   int convert(unsigned long& id, unsigned long& time, STrackedObject& obj, char* buf, size_t buflen, int op) {
      bool r = true;
      XDR xdrs;
      bool_t b = obj.valid;
      xdrmem_create(&xdrs, buf, buflen, (xdr_op)op);
      r = r and xdr_u_long(&xdrs, (unsigned long*)&id) == 1;
      r = r and xdr_u_long(&xdrs, (unsigned long*)&time) == 1;
      r = r and xdr_bool(&xdrs, &b) == 1;
      r = r and xdr_float(&xdrs, &obj.x) == 1;
      r = r and xdr_float(&xdrs, &obj.y) == 1;
      r = r and xdr_float(&xdrs, &obj.z) == 1;
      r = r and xdr_float(&xdrs, &obj.pitch) == 1;
      r = r and xdr_float(&xdrs, &obj.roll) == 1;
      r = r and xdr_float(&xdrs, &obj.yaw) == 1;
      r = r and xdr_float(&xdrs, &obj.pixel_ratio) == 1;
      r = r and xdr_float(&xdrs, &obj.bw_ratio) == 1;
      if (op == XDR_DECODE) {
	 obj.valid = b;
      }
      int ret = r ? xdr_getpos(&xdrs) : -1;
      xdr_destroy(&xdrs);
      return ret;
   }

} // end namespace imr

/* end of serialization.cc */
