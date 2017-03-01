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

  sc_interface.cpp -- Abstract base class of all interface classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/communication/sc_interface.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/kernel/sc_event.h"


namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_interface
//
//  Abstract base class of all interface classes.
//  BEWARE: Direct inheritance from this class must be done virtual.
// ----------------------------------------------------------------------------

// register a port with this interface (does nothing by default)

void
sc_interface::register_port( sc_port_base&, const char* )
{}


// get the default event

const sc_event&
sc_interface::default_event() const
{
    SC_REPORT_WARNING( SC_ID_NO_DEFAULT_EVENT_, 0 );
    return m_never_notified;
}


// destructor (does nothing)

sc_interface::~sc_interface()
{}


// constructor (does nothing)

sc_interface::sc_interface()
{}


// special event for never notified cases, note the special name to keep
// it out of the named event structures.

sc_event sc_interface::m_never_notified(SC_KERNEL_EVENT_PREFIX);

} // namespace sc_core

// $Log: sc_interface.cpp,v $
// Revision 1.5  2011/08/26 20:45:40  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.4  2011/03/12 21:07:42  acg
//  Andy Goodrich: changes to kernel generated event support.
//
// Revision 1.3  2011/03/06 15:55:08  acg
//  Andy Goodrich: Changes for named events.
//
// Revision 1.2  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!
