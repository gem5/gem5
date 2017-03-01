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

  sc_ver.h -- Version and copyright information.

  Original Author: Stan Y. Liao, Synopsys, Inc.

 NO AUTOMATIC CHANGE LOG IS GENERATED, EXPLICIT CHANGE LOG AT END OF FILE
 *****************************************************************************/


#ifndef SC_VER_H
#define SC_VER_H

#include "sysc/kernel/sc_macros.h"               // SC_CONCAT_UNDERSCORE_
                                                 // SC_STRINGIFY_HELPER_
#include "sysc/communication/sc_writer_policy.h" // SC_DEFAULT_WRITER_POLICY

#include <string>

namespace sc_core {

extern const char* sc_copyright();
extern const char* sc_release();
extern const char* sc_version();

extern const unsigned int sc_version_major;
extern const unsigned int sc_version_minor;
extern const unsigned int sc_version_patch;

extern const std::string  sc_version_originator;
extern const std::string  sc_version_release_date;
extern const std::string  sc_version_prerelease;
extern const bool         sc_is_prerelease;
extern const std::string  sc_version_string;
extern const std::string  sc_copyright_string;

#define SYSTEMC_2_3_1
 
#define SYSTEMC_VERSION       20140417
#define SC_VERSION_ORIGINATOR "Accellera"
#define SC_VERSION_MAJOR      2
#define SC_VERSION_MINOR      3
#define SC_VERSION_PATCH      1
#define SC_IS_PRERELEASE      0

/// compliancy with IEEE 1666-2011 (see 8.6.5)
#define IEEE_1666_SYSTEMC     201101L

#define SC_COPYRIGHT                               \
  "Copyright (c) 1996-2014 by all Contributors,\n" \
  "ALL RIGHTS RESERVED\n"


#define SC_VERSION_RELEASE_DATE \
  SC_STRINGIFY_HELPER_( SYSTEMC_VERSION )

#if ( SC_IS_PRERELEASE == 1 )
#  define SC_VERSION_PRERELEASE "pub_rev"
#  define SC_VERSION \
    SC_STRINGIFY_HELPER_( SC_VERSION_MAJOR.SC_VERSION_MINOR.SC_VERSION_PATCH ) \
    "_" SC_VERSION_PRERELEASE "_" SC_VERSION_RELEASE_DATE \
    "-" SC_VERSION_ORIGINATOR
#else
#  define SC_VERSION_PRERELEASE "" // nothing
#  define SC_VERSION \
    SC_STRINGIFY_HELPER_( SC_VERSION_MAJOR.SC_VERSION_MINOR.SC_VERSION_PATCH ) \
    "-" SC_VERSION_ORIGINATOR
#endif

// THIS CLASS AND STATIC INSTANCE BELOW DETECTS BAD REV OBJECTS AT LINK TIME
//
// Each source file which includes this file for the current SystemC version 
// will have a static instance of the class sc_api_version_XXX defined
// in it. That object instance will cause the constructor below
// to be invoked. If the version of the SystemC being linked against
// does not contain the constructor below a linkage error will occur.

#define SC_API_VERSION_STRING \
      SC_CONCAT_UNDERSCORE_( sc_api_version, \
      SC_CONCAT_UNDERSCORE_( SC_VERSION_MAJOR, \
      SC_CONCAT_UNDERSCORE_( SC_VERSION_MINOR, \
                             SC_VERSION_PATCH ) ) )

// explicitly avoid macro expansion
#define SC_API_DEFINED_( Symbol ) \
  Symbol ## _DEFINED_
#define SC_API_UNDEFINED_( Symbol ) \
  Symbol ## _UNDEFINED_

// Some preprocessor switches need to be consistent between the application
// and the library (e.g. if sizes of classes are affected or other parts of
// the ABI are affected).  (Some of) these are checked here at link-time as
// well, by setting template parameters to sc_api_version_XXX, while only
// one variant is defined in sc_ver.cpp.

#if 0 // don't enforce check of DEBUG_SYSTEMC for now
// DEBUG_SYSTEMC
#if defined( DEBUG_SYSTEMC )
# define DEBUG_SYSTEMC_CHECK_ \
    SC_CONFIG_DEFINED_(DEBUG_SYSTEMC)
#else
# define DEBUG_SYSTEMC_CHECK_ \
    SC_CONFIG_UNDEFINED_(DEBUG_SYSTEMC)
#endif
extern const int DEBUG_SYSTEMC_CHECK_;
#endif

// SC_DISABLE_VIRTUAL_BIND
#if defined( SC_DISABLE_VIRTUAL_BIND )
# define SC_DISABLE_VIRTUAL_BIND_CHECK_ \
    SC_API_DEFINED_(SC_DISABLE_VIRTUAL_BIND)
#else
# define SC_DISABLE_VIRTUAL_BIND_CHECK_ \
    SC_API_UNDEFINED_(SC_DISABLE_VIRTUAL_BIND)
#endif
extern const int SC_DISABLE_VIRTUAL_BIND_CHECK_;

// Some preprocessor switches need to be consistent between different
// translation units of an application.  Those can't be easily checked
// during link-time.  Instead, perform a check during run-time by
// passing the value to the constructor of the api_version_check object.

// Note: Template and constructor parameters are not passed as default
//       values to avoid ODR violations in the check itself.

template // use pointers for more verbose error messages
<
//  const int * DebugSystemC,
  const int * DisableVirtualBind
>
struct SC_API_VERSION_STRING
{
  SC_API_VERSION_STRING
    (
       // SC_DEFAULT_WRITER_POLICY
       sc_writer_policy default_writer_policy
    );
};

#if !defined( SC_DISABLE_API_VERSION_CHECK ) // disabled in sc_ver.cpp
static
SC_API_VERSION_STRING
<
//  & DEBUG_SYSTEMC_CHECK_,
  & SC_DISABLE_VIRTUAL_BIND_CHECK_
>
api_version_check
(
  SC_DEFAULT_WRITER_POLICY
);
#endif // SC_DISABLE_API_VERSION_CHECK

//#undef SC_API_DEFINED_
//#undef SC_API_UNDEFINED_

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
#endif
