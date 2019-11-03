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

  sc_boost.h -- Thread Semantics Provided By The Boost Library

  Original Author: Stuart Swan, Cadence Design Systems, Inc

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_BOOST_H
#define SC_BOOST_H

// namespace sc_dp { This is off because of bugs with gcc 2.9x

// SET THE NAME OF OBJECTS THAT THE SC_BOOST LIBRARY WILL PRODUCE AND INCLUDE IT

#if defined(_MSC_VER) && !defined(__ICL) && !defined(__COMO__)
#   pragma warning(disable: 4786)  // identifier truncated in debug info
#   pragma warning(disable: 4710)  // function not inlined  
#   pragma warning(disable: 4711)  // funct. selected for auto-inline expansion
#   pragma warning(disable: 4514)  // unreferenced inline removed
#endif

#include <functional>

#if defined(SC_BOOST_MSVC) && (SC_BOOST_MSVC < 1300)
#   pragma warning(push, 3)
#endif

#if defined(SC_BOOST_MSVC) && (SC_BOOST_MSVC < 1300)
#   pragma warning(pop)
#endif

// } // namespace sc_dp This is off because of bugs with gcc 2.9x

// macros to help avoid direct user code dependencies on boost lib
//
// note the use of the sc_boost namespace for the SystemC version of
// boost. to replace the version shipped with SystemC with another boost
// you will need to change the namespace prefix back to boost.

#define sc_bind    std::bind
#define sc_ref(r)  std::ref(r)
#define sc_cref(r) std::cref(r)

// $Log: sc_boost.h,v $
// Revision 1.7  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.6  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.5  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.4  2009/07/28 01:10:53  acg
//  Andy Goodrich: updates for 2.3 release candidate.
//
// Revision 1.3  2009/02/28 00:26:58  acg
//  Andy Goodrich: changed boost name space to sc_boost to allow use with
//  full boost library applications.
//
// Revision 1.2  2008/05/22 17:06:24  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//

#endif // SC_BOOST_H
