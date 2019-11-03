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

  sc_communication_ids.h -- Report ids for the communication code.

  Original Author: Martin Janssen, Synopsys, Inc., 2002-01-17

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_COMMUNICATION_IDS_H
#define SC_COMMUNICATION_IDS_H


#include "sysc/utils/sc_report.h"


// ----------------------------------------------------------------------------
//  Report ids (communication)
//
//  Report ids in the range of 100-199.
// ----------------------------------------------------------------------------

#ifndef SC_DEFINE_MESSAGE
#define SC_DEFINE_MESSAGE(id,unused1,unused2) \
    namespace sc_core { extern const char id[]; }
namespace sc_core {
    extern const char SC_ID_REGISTER_ID_FAILED_[]; // in sc_report_handler.cpp
} // namespace sc_core
#endif

SC_DEFINE_MESSAGE( SC_ID_PORT_OUTSIDE_MODULE_, 100,
			"port specified outside of module" )
SC_DEFINE_MESSAGE( SC_ID_CLOCK_PERIOD_ZERO_, 101,
			"sc_clock period is zero" )              
SC_DEFINE_MESSAGE( SC_ID_CLOCK_HIGH_TIME_ZERO_, 102,
			"sc_clock high time is zero" )    
SC_DEFINE_MESSAGE( SC_ID_CLOCK_LOW_TIME_ZERO_, 103,
			"sc_clock low time is zero" )     
SC_DEFINE_MESSAGE( SC_ID_MORE_THAN_ONE_FIFO_READER_, 104,
			"sc_fifo<T> cannot have more than one reader" )
SC_DEFINE_MESSAGE( SC_ID_MORE_THAN_ONE_FIFO_WRITER_, 105,
			"sc_fifo<T> cannot have more than one writer" )
SC_DEFINE_MESSAGE( SC_ID_INVALID_FIFO_SIZE_, 106,
			"sc_fifo<T> must have a size of at least 1" )
SC_DEFINE_MESSAGE( SC_ID_BIND_IF_TO_PORT_, 107,
			"bind interface to port failed" ) 
SC_DEFINE_MESSAGE( SC_ID_BIND_PORT_TO_PORT_, 108,
			"bind parent port to port failed" )
SC_DEFINE_MESSAGE( SC_ID_COMPLETE_BINDING_, 109,
			"complete binding failed" )
SC_DEFINE_MESSAGE( SC_ID_INSERT_PORT_, 110,
			"insert port failed" )
SC_DEFINE_MESSAGE( SC_ID_REMOVE_PORT_, 111,
			"remove port failed" )
SC_DEFINE_MESSAGE( SC_ID_GET_IF_, 112,
			"get interface failed" )
SC_DEFINE_MESSAGE( SC_ID_INSERT_PRIM_CHANNEL_, 113,
			"insert primitive channel failed" )
SC_DEFINE_MESSAGE( SC_ID_REMOVE_PRIM_CHANNEL_, 114, 
			"remove primitive channel failed" ) 
SC_DEFINE_MESSAGE( SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_, 115,
			"sc_signal<T> cannot have more than one driver" )
SC_DEFINE_MESSAGE( SC_ID_NO_DEFAULT_EVENT_,    116,
			"channel doesn't have a default event" )
SC_DEFINE_MESSAGE( SC_ID_RESOLVED_PORT_NOT_BOUND_, 117,
			"resolved port not bound to resolved signal" )
SC_DEFINE_MESSAGE( SC_ID_FIND_EVENT_, 118,
			"find event failed" )
SC_DEFINE_MESSAGE( SC_ID_INVALID_SEMAPHORE_VALUE_,  119,
			"sc_semaphore requires an initial value >= 0" )
SC_DEFINE_MESSAGE( SC_ID_SC_EXPORT_HAS_NO_INTERFACE_,  120,
			"sc_export instance has no interface" )
SC_DEFINE_MESSAGE( SC_ID_INSERT_EXPORT_,  121,
    "insert sc_export failed" )
SC_DEFINE_MESSAGE( SC_ID_SC_EXPORT_NOT_REGISTERED_,  123,
    "remove sc_export failed, sc_export not registered" )
SC_DEFINE_MESSAGE( SC_ID_SC_EXPORT_NOT_BOUND_AFTER_CONSTRUCTION_,  124,
    "sc_export instance not bound to interface at end of construction" )
SC_DEFINE_MESSAGE( SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_,  125,
   "attempt to write the value of an sc_clock instance" )
SC_DEFINE_MESSAGE( SC_ID_SC_EXPORT_ALREADY_BOUND_,  126,
    "sc_export instance already bound" )
SC_DEFINE_MESSAGE( SC_ID_OPERATION_ON_NON_SPECIALIZED_SIGNAL_,  127,
    "attempted specalized signal operation on non-specialized signal" )
SC_DEFINE_MESSAGE( SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_,  128,
    "attempted to bind sc_clock instance to sc_inout or sc_out" )
SC_DEFINE_MESSAGE( SC_ID_NO_ASYNC_UPDATE_,  129,
    "this build has no asynchronous update support" )

/* 
$Log: sc_communication_ids.h,v $
Revision 1.5  2011/08/26 20:45:39  acg
 Andy Goodrich: moved the modification log to the end of the file to
 eliminate source line number skew when check-ins are done.

Revision 1.4  2011/04/19 02:36:26  acg
 Philipp A. Hartmann: new aysnc_update and mutex support.

Revision 1.3  2011/02/18 20:23:45  acg
 Andy Goodrich: Copyright update.

Revision 1.2  2011/02/14 17:50:16  acg
 Andy Goodrich: testing for sc_port and sc_export instantiations during
 end of elaboration and issuing appropriate error messages.

Revision 1.1.1.1  2006/12/15 20:20:04  acg
SystemC 2.3

Revision 1.5  2006/01/25 00:31:11  acg
 Andy Goodrich: Changed over to use a standard message id of
 SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.

Revision 1.4  2006/01/24 20:46:31  acg
Andy Goodrich: changes to eliminate use of deprecated features. For instance,
using notify(SC_ZERO_TIME) in place of notify_delayed().

Revision 1.3  2006/01/18 21:42:26  acg
Andy Goodrich: Changes for check writer support, and tightening up sc_clock
port usage.

Revision 1.2  2006/01/03 23:18:26  acg
Changed copyright to include 2006.

Revision 1.1.1.1  2005/12/19 23:16:43  acg
First check in of SystemC 2.1 into its own archive.

Revision 1.12  2005/04/03 22:52:51  acg
Namespace changes.

Revision 1.11  2005/03/21 22:31:32  acg
Changes to sc_core namespace.

Revision 1.10  2004/10/28 00:21:48  acg
Added check that sc_export instances are not bound twice.

Revision 1.9  2004/09/27 21:02:54  acg
Andy Goodrich - Forte Design Systems, Inc.
   - Added a $Log comment so that CVS checkin comments will appear in
     checked out source.

*/

#endif

// Taf!
