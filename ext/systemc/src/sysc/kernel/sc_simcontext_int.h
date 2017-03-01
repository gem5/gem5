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

  sc_simcontext_int.h -- For inline definitions of some utility functions.
                         DO NOT EXPORT THIS INCLUDE FILE. Include this file
                         after "sc_process_int.h" so that we can get the base
                         class right.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_SIMCONTEXT_INT_H
#define SC_SIMCONTEXT_INT_H

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_runnable.h"
#include "sysc/kernel/sc_runnable_int.h"

// DEBUGGING MACROS:
//
// DEBUG_MSG(NAME,P,MSG)
//     MSG  = message to print
//     NAME = name that must match the process for the message to print, or
//            null if the message should be printed unconditionally.
//     P    = pointer to process message is for, or NULL in which case the
//            message will not print.
#if 0
#   define DEBUG_NAME ""
#   define DEBUG_MSG(NAME,P,MSG) \
    { \
        if ( P && ( (strlen(NAME)==0) || !strcmp(NAME,P->name())) ) \
          std::cout << "**** " << sc_time_stamp() << " ("  \
	            << sc_get_current_process_name() << "): " << MSG \
		    << " - " << P->name() << std::endl; \
    }
#else
#   define DEBUG_MSG(NAME,P,MSG) 
#endif


namespace sc_core {

inline
const char*
sc_get_current_process_name()
{
    sc_process_b* active_p; // active process to get name of.
    const char*   result;   // name of active process.

    active_p = sc_get_curr_simcontext()->get_curr_proc_info()->process_handle;
    if ( active_p )
        result = active_p->name();
    else
        result = "** NONE **";
    return result;
}

// We use m_current_writer rather than m_curr_proc_info.process_handle to
// return the active process for sc_signal<T>::check_write since that lets
// us turn it off a library compile time, and only incur the overhead at
// the time of process switches rather than having to interrogate an
// additional switch every time a signal is written.

inline
void
sc_simcontext::set_curr_proc( sc_process_b* process_h )
{
    m_curr_proc_info.process_handle = process_h;
    m_curr_proc_info.kind           = process_h->proc_kind();
    m_current_writer = m_write_check ? process_h : (sc_object*)0;
}

inline
void
sc_simcontext::reset_curr_proc()
{
    m_curr_proc_info.process_handle = 0;
    m_curr_proc_info.kind           = SC_NO_PROC_;
    m_current_writer                = 0;
    sc_process_b::m_last_created_process_p = 0; 
}

inline
void
sc_simcontext::execute_method_next( sc_method_handle method_h )
{
    m_runnable->execute_method_next( method_h );
}

inline
void
sc_simcontext::execute_thread_next( sc_thread_handle thread_h )
{
    m_runnable->execute_thread_next( thread_h );
}

// +----------------------------------------------------------------------------
// |"sc_simcontext::preempt_with"
// | 
// | This method executes the supplied thread immediately, suspending the
// | caller. After executing the supplied thread the caller's execution will
// | be restored. It is used to allow a thread to immediately throw an 
// | exception, e.g., when the thread's kill_process() method was called.
// | There are three cases to consider:
// |   (1) The caller is a method, e.g., murder by method.
// |   (2) The caller is another thread instance, e.g., murder by thread.
// |   (3) The caller is this thread instance, e.g., suicide.
// |
// | Arguments:
// |     thread_h -> thread to be executed.
// +----------------------------------------------------------------------------
inline
void
sc_simcontext::preempt_with( sc_thread_handle thread_h )
{
    sc_thread_handle  active_p;    // active thread or null.
    sc_curr_proc_info caller_info; // process info for caller.

    // Determine the active process and take the thread to be run off the
    // run queue, if its there, since we will be explicitly causing its 
    // execution.

    active_p = DCAST<sc_thread_handle>(sc_get_current_process_b());
    if ( thread_h->next_runnable() != NULL )
	remove_runnable_thread( thread_h );

    // THE CALLER IS A METHOD:
    //
    //   (a) Set the current process information to our thread.
    //   (b) If the method was called by an invoker thread push that thread
    //       onto the front of the run queue, this will cause the method
    //       to be resumed after this thread waits.
    //   (c) Invoke our thread directly by-passing the run queue.
    //   (d) Restore the process info to the caller.
    //   (e) Check to see if the calling method should throw an exception
    //       because of activity that occurred during the preemption.

    if ( active_p == NULL )
    {
	std::vector<sc_thread_handle>* invokers_p;  // active invokers stack.
	sc_thread_handle           invoke_thread_p; // latest invocation thread.
        sc_method_handle           method_p;        // active method.

	method_p = DCAST<sc_method_handle>(sc_get_current_process_b());
	invokers_p = &get_active_invokers();
	caller_info = m_curr_proc_info;
	if ( invokers_p->size() != 0 )
	{
	    invoke_thread_p = invokers_p->back();
	    DEBUG_MSG( DEBUG_NAME, invoke_thread_p, 
	        "queueing invocation thread to execute next" );
	    execute_thread_next(invoke_thread_p);
	}
        DEBUG_MSG( DEBUG_NAME, thread_h, "preempting method with thread" );
	set_curr_proc( (sc_process_b*)thread_h );
	m_cor_pkg->yield( thread_h->m_cor_p );
	m_curr_proc_info = caller_info; 
        DEBUG_MSG(DEBUG_NAME, thread_h, "back from preempting method w/thread");
	method_p->check_for_throws();
    }

    // CALLER IS A THREAD, BUT NOT THE THREAD TO BE RUN:
    //
    //   (a) Push the calling thread onto the front of the runnable queue
    //       so it be the first thread to be run after this thread.
    //   (b) Push the thread to be run onto the front of the runnable queue so 
    //       it will execute when we suspend the calling thread.
    //   (c) Suspend the active thread.

    else if ( active_p != thread_h )
    {
        DEBUG_MSG( DEBUG_NAME, thread_h,
	           "preempting active thread with thread" );
        execute_thread_next( active_p );
	execute_thread_next( thread_h );
	active_p->suspend_me();
    }

    // CALLER IS THE THREAD TO BE RUN:
    //
    //   (a) Push the thread to be run onto the front of the runnable queue so 
    //       it will execute when we suspend the calling thread.
    //   (b) Suspend the active thread.

    else
    {
        DEBUG_MSG(DEBUG_NAME,thread_h,"self preemption of active thread");
	execute_thread_next( thread_h );
	active_p->suspend_me();
    }
}


inline
void
sc_simcontext::push_runnable_method( sc_method_handle method_h )
{
    m_runnable->push_back_method( method_h );
}

inline
void
sc_simcontext::push_runnable_method_front( sc_method_handle method_h )
{
    m_runnable->push_front_method( method_h );
}

inline
void
sc_simcontext::push_runnable_thread( sc_thread_handle thread_h )
{
    m_runnable->push_back_thread( thread_h );
}

inline
void
sc_simcontext::push_runnable_thread_front( sc_thread_handle thread_h )
{
    m_runnable->push_front_thread( thread_h );
}


inline
sc_method_handle
sc_simcontext::pop_runnable_method()
{
    sc_method_handle method_h = m_runnable->pop_method();
    if( method_h == 0 ) {
	reset_curr_proc();
	return 0;
    }
    set_curr_proc( (sc_process_b*)method_h );
    return method_h;
}

inline
sc_thread_handle
sc_simcontext::pop_runnable_thread()
{
    sc_thread_handle thread_h = m_runnable->pop_thread();
    if( thread_h == 0 ) {
	reset_curr_proc();
	return 0;
    }
    set_curr_proc( (sc_process_b*)thread_h );
    return thread_h;
}

inline
void
sc_simcontext::remove_runnable_method( sc_method_handle method_h )
{
    m_runnable->remove_method( method_h );
}

inline
void
sc_simcontext::remove_runnable_thread( sc_thread_handle thread_h )
{
    m_runnable->remove_thread( thread_h );
}

inline
std::vector<sc_thread_handle>&
sc_simcontext::get_active_invokers()
{
    return m_active_invokers;
}

// ----------------------------------------------------------------------------

extern void sc_defunct_process_function( sc_module* );


} // namespace sc_core

#undef DEBUG_MSG
#undef DEBUG_NAME

// $Log: sc_simcontext_int.h,v $
// Revision 1.14  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.13  2011/08/26 20:46:11  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.12  2011/07/29 22:45:06  acg
//  Andy Goodrich: added invocation of sc_method_process::check_for_throws()
//  to the preempt_with() code to handle case where the preempting process
//  causes a throw on the invoking method process.
//
// Revision 1.11  2011/04/13 02:45:11  acg
//  Andy Goodrich: eliminated warning message that occurred if the DEBUG_MSG
//  macro was used.
//
// Revision 1.10  2011/04/11 22:05:48  acg
//  Andy Goodrich: use the DEBUG_NAME macro in DEBUG_MSG invocations.
//
// Revision 1.9  2011/04/10 22:12:32  acg
//  Andy Goodrich: adding debugging macros.
//
// Revision 1.8  2011/04/08 18:26:07  acg
//  Andy Goodrich: added execute_method_next() to handle method dispatch
//   for asynchronous notifications that occur outside the evaluation phase.
//
// Revision 1.7  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.6  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.5  2011/02/08 08:17:50  acg
//  Andy Goodrich: fixed bug in preempt_with() where I was resetting the
//  process context rather than saving and restoring it.
//
// Revision 1.4  2011/02/01 21:12:56  acg
//  Andy Goodrich: addition of preempt_with() method to allow immediate
//  execution of threads for throws.
//
// Revision 1.3  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.2  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.6  2006/05/26 20:33:16  acg
//   Andy Goodrich: changes required by additional platform compilers (i.e.,
//   Microsoft VC++, Sun Forte, HP aCC).
//
// Revision 1.5  2006/01/19 00:29:52  acg
// Andy Goodrich: Yet another implementation for signal write checking. This
// one uses an environment variable SC_SIGNAL_WRITE_CHECK, that when set to
// DISABLE will disable write checking on signals.
//
// Revision 1.4  2006/01/18 21:42:37  acg
// Andy Goodrich: Changes for check writer support.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.

#endif
