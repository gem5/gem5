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

  sc_cmnhdr.h - Common header file containing handy pragmas, macros and
                definitions common to all SystemC source files.

  Original Author: Amit Rao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#ifndef SC_CMNHDR_H
#define SC_CMNHDR_H

#if defined(_WIN32) || defined(_MSC_VER) || defined(__BORLANDC__) || \
	defined(__MINGW32__)

// all windows 32-bit compilers should define WIN32
#if !defined(WIN32) && !defined(WIN64) && !defined(_WIN64)
#define WIN32
#endif

// Windows Version Build Option
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0400
#endif

// remember to later include windows.h, if needed
#define SC_HAS_WINDOWS_H_

#endif // WIN32

// ----------------------------------------------------------------------------

#ifdef _MSC_VER

// Disable VC++ warnings that are harmless

// this : used in base member initializer list
#pragma warning(disable: 4355)

// new and delete warning when exception handling is turned on
#pragma warning(disable: 4291)

// in many places implicit conversion to bool
// from other integral types is performed
#pragma warning(disable: 4800)

// unary minus operator applied to unsigned
#pragma warning(disable: 4146)

// multiple copy constructors
#pragma warning(disable: 4521)

// identifier was truncated to '255' characters in the browser information
#pragma warning(disable: 4786)

#endif 

// ----------------------------------------------------------------------------
// helper macros to aid branch prediction on GCC (compatible) compilers

#ifndef __GNUC__
#  define SC_LIKELY_( x )    !!(x)
#  define SC_UNLIKELY_( x )  !!(x)
#else
#  define SC_LIKELY_( x )    __builtin_expect( !!(x), 1 )
#  define SC_UNLIKELY_( x )  __builtin_expect( !!(x), 0 )
#endif

// ----------------------------------------------------------------------------

#include <cassert>
#include <cstdio>
#include <cstdlib>

#endif // SC_CMNHDR_H

// ----------------------------------------------------------------------------
// only include Windows.h, if explicitly requested
// (deliberately outside of include guards to enable later effect)
#if defined(SC_HAS_WINDOWS_H_) && defined(SC_INCLUDE_WINDOWS_H)
#  undef SC_HAS_WINDOWS_H_
#  include <Windows.h>
#endif

// $Log: sc_cmnhdr.h,v $
// Revision 1.8  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.7  2011/05/09 04:07:48  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.6  2011/05/05 17:45:27  acg
//  Philip A. Hartmann: changes in WIN64 support.
//  Andy Goodrich: additional DEBUG_MSG instances to trace process handling.
//
// Revision 1.5  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.4  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.3  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.2  2008/05/22 17:06:24  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

// Taf!
