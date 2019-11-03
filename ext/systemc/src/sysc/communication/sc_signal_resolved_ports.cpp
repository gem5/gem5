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

  sc_signal_resolved_ports.cpp -- The sc_signal_resolved port classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-08-20

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include <cstdio>

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/communication/sc_signal_resolved.h"
#include "sysc/communication/sc_signal_resolved_ports.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_in_resolved
//
//  The sc_signal_resolved input port class.
// ----------------------------------------------------------------------------

// called when elaboration is done

void
sc_in_resolved::end_of_elaboration()
{
    base_type::end_of_elaboration();
    // check if bound channel is a resolved signal
    if( DCAST<sc_signal_resolved*>( get_interface() ) == 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg, "%s (%s)", name(), kind() );
	SC_REPORT_ERROR( SC_ID_RESOLVED_PORT_NOT_BOUND_, msg );
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_inout_resolved
//
//  The sc_signal_resolved input/output port class.
// ----------------------------------------------------------------------------

// called when elaboration is done

void
sc_inout_resolved::end_of_elaboration()
{
    base_type::end_of_elaboration();
    // check if bound channel is a resolved signal
    if( DCAST<sc_signal_resolved*>( get_interface() ) == 0 ) {
	char msg[BUFSIZ];
	std::sprintf( msg, "%s (%s)", name(), kind() );
	SC_REPORT_ERROR( SC_ID_RESOLVED_PORT_NOT_BOUND_, msg );
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_out_resolved
//
//  The sc_signal_resolved output port class.
// ----------------------------------------------------------------------------

} // namespace sc_core

// $Log: sc_signal_resolved_ports.cpp,v $
// Revision 1.4  2011/08/26 20:45:44  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.2  2011/02/07 19:16:50  acg
//  Andy Goodrich: changes for handling multiple writers.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!
