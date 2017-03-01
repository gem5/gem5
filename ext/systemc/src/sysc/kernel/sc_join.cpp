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

  sc_join.cpp -- Join Process Synchronization Implementation

  Original Author: Andy Goodrich, Forte Design Systems, 5 May 2003

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/


#include <cassert>
#include <cstdlib>
#include <cstddef>

#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_thread_process.h"
#include "sysc/kernel/sc_join.h"

namespace sc_core {

//------------------------------------------------------------------------------
//"sc_join::sc_join"
//
// This is the object instance constructor for this class.
//------------------------------------------------------------------------------
sc_join::sc_join()
  : m_join_event( (std::string(SC_KERNEL_EVENT_PREFIX)+"_join_event").c_str() )
  , m_threads_n(0)
{}

//------------------------------------------------------------------------------
//"sc_join::add_process - sc_process_b*"
//
// This method adds a process to this join object instance. This consists of
// incrementing the count of processes in the join process and adding this 
// object instance to the supplied thread's monitoring queue.
//     process_p -> thread to be monitored.
//------------------------------------------------------------------------------
void sc_join::add_process( sc_process_b* process_p )
{
    sc_thread_handle handle = DCAST<sc_thread_handle>(process_p);
    assert( handle != 0 );
    m_threads_n++;
    handle->add_monitor( this );
}


//------------------------------------------------------------------------------
//"sc_join::add_process - sc_process_handle"
//
// This method adds a process to this join object instance. This consists of
// incrementing the count of processes in the join process and adding this 
// object instance to the supplied thread's monitoring queue.
//     process_h = handle for process to be monitored.
//------------------------------------------------------------------------------
void sc_join::add_process( sc_process_handle process_h )
{
    sc_thread_handle thread_p; // Thread within process_h.

    thread_p = process_h.operator sc_thread_handle();
    if ( thread_p )
    {
        m_threads_n++;
        thread_p->add_monitor( this );
    }
    else
    {
        SC_REPORT_ERROR( SC_ID_JOIN_ON_METHOD_HANDLE_, 0 ); 
    }
}


//------------------------------------------------------------------------------
//"sc_join::signal"
//
// This virtual method is called when a process being monitored by this object
// instance sends a signal. If the signal type is spm_exit and the count of 
// threads that we are waiting to terminate on goes to zero we fire our join 
// event.
//     thread_p -> thread that is signalling.
//     type     =  type of signal being sent.
//------------------------------------------------------------------------------
void sc_join::signal(sc_thread_handle thread_p, int type)
{
    switch ( type )
    {
      case sc_process_monitor::spm_exit:
        thread_p->remove_monitor(this);
        if ( --m_threads_n == 0 ) m_join_event.notify();
        break;
    }
}

} // namespace sc_core

// $Log: sc_join.cpp,v $
// Revision 1.7  2011/08/26 21:45:00  acg
//  Andy Goodrich: fix internal event naming.
//
// Revision 1.6  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.5  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.4  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.3  2009/07/28 01:10:53  acg
//  Andy Goodrich: updates for 2.3 release candidate.
//
// Revision 1.2  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.
//
