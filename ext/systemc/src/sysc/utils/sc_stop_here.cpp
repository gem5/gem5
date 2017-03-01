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

  sc_stop_here.cpp -- Function provided for debugging purposes.
                      This file is always compiled in debug mode, such that
                      setting a breakpoint at this function can help locate
                      the cause of a SystemC error or warning.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-11-14

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#include "sysc/utils/sc_stop_here.h"


namespace sc_core {

static const char* info_id    = 0;
static const char* warning_id = 0;
static const char* error_id   = 0;
static const char* fatal_id   = 0;

// ----------------------------------------------------------------------------
//  FUNCTION : sc_interrupt_here
//
//  Debugging aid for warning, error, and fatal reports.
//  This function *cannot* be inlined.
// ----------------------------------------------------------------------------

void
sc_interrupt_here( const char* id, sc_severity severity )
{
    // you can set a breakpoint at some of the lines below, either to
    // interrupt with any severity, or to interrupt with a specific severity

    switch( severity ) {
      case SC_INFO: 
	info_id = id;
	break;
      case SC_WARNING: 
	warning_id = id;
	break;
      case SC_ERROR: 
	error_id = id;
	break;
      default:
      case SC_FATAL: 
	fatal_id = id;
	break;
    }
}


// ----------------------------------------------------------------------------
//  FUNCTION : sc_stop_here
//
//  Debugging aid for warning, error, and fatal reports.
//  This function *cannot* be inlined.
// ----------------------------------------------------------------------------

void
sc_stop_here( const char* id, sc_severity severity )
{
    // you can set a breakpoint at some of the lines below, either to
    // stop with any severity, or to stop with a specific severity

    switch( severity ) {
      case SC_INFO: 
	info_id = id;
	break;
      case SC_WARNING: 
	warning_id = id;
	break;
      case SC_ERROR: 
	error_id = id;
	break;
      default:
      case SC_FATAL: 
	fatal_id = id;
	break;
    }
}

} // namespace sc_core

// $Log: sc_stop_here.cpp,v $
// Revision 1.4  2011/08/26 21:49:08  acg
//  Philipp A. Hartmann: eliminate compiler warning by moving static variables
//  out of functions.
//
// Revision 1.3  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// Taf!
