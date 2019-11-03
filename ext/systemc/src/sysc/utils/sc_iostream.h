/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_iostream.h - Portable iostream header file wrapper.

  Original Author: Martin Janssen, Synopsys, Inc.

  Note: Deprecated in the meantime, since all supported
        compilers are supposed to have a working C++
        standard library.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#ifndef SC_IOSTREAM_H
#define SC_IOSTREAM_H

#include <ios>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstddef>
#include <cstring>

// We use typedefs for istream and ostream here to get around some finickiness
// from aCC:

namespace sc_dt {

typedef ::std::istream systemc_istream;
typedef ::std::ostream systemc_ostream;

} // namespace sc_dt

// shortcuts that save some typing

#ifdef CCAST
#    undef CCAST
#endif
#define CCAST       const_cast

#ifdef DCAST
#    undef DCAST
#endif
#define DCAST     dynamic_cast

#ifdef RCAST
#    undef RCAST
#endif
#define RCAST reinterpret_cast

#ifdef SCAST
#    undef SCAST
#endif
#define SCAST      static_cast

// $Log: sc_iostream.h,v $
// Revision 1.3  2011/08/26 20:46:18  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.2  2011/02/18 20:38:43  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:10  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif // !defined(SC_IOSTREAM_H)
