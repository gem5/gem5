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

    sc_ver.cpp -- copyright information.

    Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#include <cstddef>
#include <cstdlib>

#define SC_DISABLE_API_VERSION_CHECK // for in-library sc_ver.h inclusion

#include "sysc/kernel/sc_ver.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/utils/sc_iostream.h"
#include "sysc/utils/sc_report.h"

using std::getenv;
using std::strcmp;
using std::cerr;
using std::endl;

namespace sc_core {


static
const char systemc_version[] =
    "SystemC " SC_VERSION " --- " __DATE__ " " __TIME__;

const unsigned int sc_version_major = SC_VERSION_MAJOR;
const unsigned int sc_version_minor = SC_VERSION_MINOR;
const unsigned int sc_version_patch = SC_VERSION_PATCH;
const bool         sc_is_prerelease = SC_IS_PRERELEASE;

const std::string  sc_version_originator   = SC_VERSION_ORIGINATOR;
const std::string  sc_version_release_date = SC_VERSION_RELEASE_DATE;
const std::string  sc_version_prerelease   = SC_VERSION_PRERELEASE;
const std::string  sc_version_string       = SC_VERSION;
const std::string  sc_copyright_string     = SC_COPYRIGHT;

const char*
sc_copyright()
{
    return SC_COPYRIGHT;
}


const char*
sc_release()
{
    return SC_VERSION;
}


const char*
sc_version()
{
    return systemc_version;
}


#if !defined(SC_DISABLE_COPYRIGHT_MESSAGE)
#  define SC_DISABLE_COPYRIGHT_MESSAGE 0
#endif

// ----------------------------------------------------------------------------

void
pln()
{
    static bool lnp = SC_DISABLE_COPYRIGHT_MESSAGE;
    if ( lnp || getenv("SYSTEMC_DISABLE_COPYRIGHT_MESSAGE") != 0 ) 
        lnp = true;
    if ( const char * lnp_env = getenv("SC_COPYRIGHT_MESSAGE") ) {
        lnp = !strcmp( lnp_env, "DISABLE" );
    }
    if( ! lnp ) {

        static const char indent[] = "        ";
        std::string       line;
        std::stringstream copyright;

        // temporary stream to print copyright line-wise with indentation
        copyright << sc_copyright();

        cerr << endl;
        cerr << indent << sc_version() << endl;
        while( getline( copyright, line ) )
            cerr << indent << line << endl;

        //  regressions check point

        if( getenv( "SYSTEMC_REGRESSION" ) != 0 ) {
            cerr << "SystemC Simulation" << endl;
        }

        lnp = true;
    }
}

#define SC_API_PERFORM_CHECK_( Type, Name, Symbol ) \
  do { \
    static bool SC_CONCAT_UNDERSCORE_( Name, config_seen ) = false; \
    static Type SC_CONCAT_UNDERSCORE_( Name, config ); \
    if( ! SC_CONCAT_UNDERSCORE_( Name, config_seen ) ) { \
      SC_CONCAT_UNDERSCORE_( Name, config_seen ) = true; \
      SC_CONCAT_UNDERSCORE_( Name, config ) = Name; \
    } else if( SC_CONCAT_UNDERSCORE_( Name, config ) != Name ) { \
      SC_REPORT_FATAL( SC_ID_INCONSISTENT_API_CONFIG_, Symbol ); \
    } \
  } while( false )

// THIS CONSTRUCTOR ROOTS OUT OLD OBJECTS AT LINK TIME
//
// Each source file which includes sc_ver.h for this SystemC version 
// will have a static instance of the class sc_api_version_XXX defined 
// in it. That object instanciation will cause the constructor below 
// to be invoked. If the version of the SystemC being linked against
// does not contain the constructor below a linkage error will occur.
//
// Some preprocessor switches need to be consistent between the application
// and the library (e.g. if sizes of classes are affected or other parts of
// the ABI are affected).  (Some of) these are checked here at link-time as
// well, by setting template parameters to sc_api_version_XXX, while only
// one variant is defined here.
//
// Some preprocessor switches need to be consistent between different
// translation units of an application.  Those can't be easily checked
// during link-time.  Instead, perform a check during run-time by
// passing the value to the constructor of the api_version_check object.

// const int DEBUG_SYSTEMC_CHECK_           = 1;
const int SC_DISABLE_VIRTUAL_BIND_CHECK_ = 1;

template<>
SC_API_VERSION_STRING
<
//   & DEBUG_SYSTEMC_CHECK_,
  & SC_DISABLE_VIRTUAL_BIND_CHECK_
>
::SC_API_VERSION_STRING
(
  sc_writer_policy default_writer_policy
)
{
  SC_API_PERFORM_CHECK_( sc_writer_policy
                          , default_writer_policy
                          , "SC_DEFAULT_WRITER_POLICY" );
}

} // namespace sc_core

// $Log: sc_ver.cpp,v $
// Revision 1.14  2011/08/26 21:56:55  acg
//  Torsten Maehne: use usings rather than absolute namespace addressing.
//
// Revision 1.13  2011/08/26 20:46:11  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.12  2011/07/25 10:20:34  acg
//  Andy Goodrich: check in aftermath of call to automake.
//
// Revision 1.11  2011/07/02 12:55:19  acg
//  Andy Goodrich: automake refresh.
//
// Revision 1.10  2011/07/01 18:49:07  acg
//  Andy Goodrich: moved pln() from sc_simcontext.cpp to sc_ver.cpp.
//
// Revision 1.9  2011/07/01 18:33:08  acg
//  Andy Goodrich: changes for IEEE 1666, removal of macros and use of them.
//
// Revision 1.8  2011/04/08 18:27:53  acg
//  Andy Goodrich: respin of the PoC.
//
// Revision 1.7  2011/04/05 20:50:57  acg
//  Andy Goodrich:
//    (1) changes to make sure that event(), posedge() and negedge() only
//        return true if the clock has not moved.
//    (2) fixes for method self-resumes.
//    (3) added SC_PRERELEASE_VERSION
//    (4) removed kernel events from the object hierarchy, added
//        sc_hierarchy_name_exists().
//
// Revision 1.6  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.5  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.4  2011/01/18 20:10:45  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.3  2010/11/20 17:10:57  acg
//  Andy Goodrich: reset processing changes for new IEEE 1666 standard.
//
// Revision 1.2  2008/05/22 17:06:27  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//

// Taf!
