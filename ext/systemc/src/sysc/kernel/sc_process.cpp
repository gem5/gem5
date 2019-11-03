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

  sc_process.cpp -- Base process implementation.

  Original Authors: Andy Goodrich, Forte Design Systems, 17 June 2003
                    Stuart Swan, Cadence
                    Bishnupriya Bhattacharya, Cadence Design Systems,
                    25 August, 2003

 CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/kernel/sc_name_gen.h"
#include "sysc/kernel/sc_cthread_process.h"
#include "sysc/kernel/sc_method_process.h"
#include "sysc/kernel/sc_thread_process.h"
#include "sysc/kernel/sc_sensitive.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_event.h"
#include <sstream>

namespace sc_core {

// sc_process_handle entities that are returned for null pointer instances:
//
// Note the special name for 'non_event' - this makes sure it does not
// appear as a named event.

std::vector<sc_event*> sc_process_handle::empty_event_vector;
std::vector<sc_object*> sc_process_handle::empty_object_vector;
sc_event                sc_process_handle::non_event(SC_KERNEL_EVENT_PREFIX);

// Last process that was created:

sc_process_b* sc_process_b::m_last_created_process_p = 0;

//------------------------------------------------------------------------------
//"sc_process_b::add_static_event"
//
// This method adds an event to the list of static events, and sets the
// event up to call back this process when it fires.
//------------------------------------------------------------------------------
void sc_process_b::add_static_event( const sc_event& e )
{
    sc_method_handle method_h; // This process as a method.
    sc_thread_handle thread_h; // This process as a thread.
    

    // CHECK TO SEE IF WE ARE ALREADY REGISTERED WITH THE EVENT:

    for( int i = m_static_events.size() - 1; i >= 0; -- i ) {
        if( &e == m_static_events[i] ) {
            return;
        }
    }

    // REMEMBER THE EVENT AND THEN REGISTER OUR OBJECT INSTANCE WITH IT:

    m_static_events.push_back( &e );

    switch ( m_process_kind )
    {
      case SC_THREAD_PROC_:
      case SC_CTHREAD_PROC_:
        thread_h = SCAST<sc_thread_handle>( this );
        e.add_static( thread_h );
        break;
      case SC_METHOD_PROC_:
        method_h = SCAST<sc_method_handle>( this );
        e.add_static( method_h );
        break;
      default:
        assert( false );
        break;
    }
}

//------------------------------------------------------------------------------
//"sc_process_b::disconnect_process"
//
// This method removes this object instance from use. It will be called by
// the kill_process() methods of classes derived from it. This object instance
// will be removed from any data structures it resides, other than existence.
//------------------------------------------------------------------------------
void sc_process_b::disconnect_process()
{
    int               mon_n;      // monitor queue size.
    sc_thread_handle  thread_h;   // This process as a thread.

    // IF THIS OBJECT IS PINING FOR THE FJORDS WE ARE DONE:

    if ( m_state & ps_bit_zombie ) return;    

    // IF THIS IS A THREAD SIGNAL ANY MONITORS WAITING FOR IT TO EXIT:

    switch ( m_process_kind )
    {
      case SC_THREAD_PROC_:
      case SC_CTHREAD_PROC_:
        thread_h = SCAST<sc_thread_handle>(this);
        mon_n = thread_h->m_monitor_q.size();
        if ( mon_n )
        {
            for ( int mon_i = 0; mon_i < mon_n; mon_i++ )
            {
                thread_h->m_monitor_q[mon_i]->signal( thread_h, 
			      sc_process_monitor::spm_exit);
            }
        }
        break;
      default:
        break;
    }

    // REMOVE EVENT WAITS, AND REMOVE THE PROCESS FROM ITS SC_RESET:

    remove_dynamic_events();
    remove_static_events();

    for ( std::vector<sc_reset*>::size_type rst_i = 0; rst_i < m_resets.size(); rst_i++ )
    {
        m_resets[rst_i]->remove_process( this );
    }
    m_resets.resize(0);


    // FIRE THE TERMINATION EVENT, MARK AS TERMINATED, AND DECREMENT THE COUNT:
    //
    // (1) We wait to set the process kind until after doing the removals
    //     above.
    // (2) Decrementing the reference count will result in actual object 
    //     deletion if we hit zero.

    m_state = ps_bit_zombie;
    if ( m_term_event_p ) m_term_event_p->notify();
    reference_decrement();
}

//------------------------------------------------------------------------------
//"sc_process_b::delete_process"
//
// This method deletes the current instance, if it is not the running
// process. Otherwise, it is put in the simcontext's process deletion
// queue.
//
// The reason for the two step deletion process is that the process from which
// reference_decrement() is called may be the running process, so we may need
// to wait until it goes idle.
//------------------------------------------------------------------------------
void sc_process_b::delete_process()
{
    assert( m_references_n == 0 );

    // Immediate deletion:

    if ( this != sc_get_current_process_b() )
    {
        delete this;
    }
  
    // Deferred deletion: note we set the reference count to one  for the call
    // to reference_decrement that occurs in sc_simcontext::crunch().
  
    else
    {
	m_references_n = 1; 
        detach();
        simcontext()->mark_to_collect_process( this );
    }
}


//------------------------------------------------------------------------------
//"sc_process_b::dont_initialize"
//
// This virtual method sets the initialization switch for this object instance.
//------------------------------------------------------------------------------
void sc_process_b::dont_initialize( bool dont )
{
    m_dont_init = dont;
}

//------------------------------------------------------------------------------
//"sc_process_b::dump_state"
//
// This method returns the process state as a string.
//------------------------------------------------------------------------------
std::string sc_process_b::dump_state() const
{
    std::string result;
    result = "[";
    if ( m_state == ps_normal ) 
    {
        result += " normal";
    }
    else
    {
        if ( m_state & ps_bit_disabled )
            result += "disabled ";
        if ( m_state & ps_bit_suspended )
            result += "suspended ";
        if ( m_state & ps_bit_ready_to_run )
            result += "ready_to_run ";
        if ( m_state & ps_bit_zombie )
            result += "zombie ";
    }
    result += "]";
    return result;
}


//------------------------------------------------------------------------------
//"sc_process_b::gen_unique_name"
//
// This method generates a unique name within this object instance's namespace.
//------------------------------------------------------------------------------
const char* sc_process_b::gen_unique_name( const char* basename_,
    bool preserve_first )
{
    if ( ! m_name_gen_p ) m_name_gen_p = new sc_name_gen;
    return m_name_gen_p->gen_unique_name( basename_, preserve_first );
}

//------------------------------------------------------------------------------
//"sc_process_b::remove_dynamic_events"
//
// This method removes this object instance from the events in its dynamic
// event lists.
//
// Arguments:
//     skip_timeout = skip cleaning up the timeout event, it will be done
//                    by sc_event_notify().
//------------------------------------------------------------------------------
void
sc_process_b::remove_dynamic_events( bool skip_timeout )
{
    sc_method_handle  method_h;   // This process as a method.
    sc_thread_handle  thread_h;   // This process as a thread.

    m_trigger_type = STATIC;
    switch ( m_process_kind )
    {
      case SC_THREAD_PROC_:
      case SC_CTHREAD_PROC_:
        thread_h = SCAST<sc_thread_handle>(this);
	if ( thread_h->m_timeout_event_p && !skip_timeout ) {
	    thread_h->m_timeout_event_p->remove_dynamic(thread_h);
	    thread_h->m_timeout_event_p->cancel();
	}
        if ( m_event_p ) m_event_p->remove_dynamic( thread_h );
        if ( m_event_list_p )
        {
            m_event_list_p->remove_dynamic( thread_h, 0 );
            m_event_list_p->auto_delete();
	    m_event_list_p = 0;
        }
        break;
      case SC_METHOD_PROC_:
        method_h = SCAST<sc_method_handle>(this);
	if ( method_h->m_timeout_event_p && !skip_timeout ) {
	    method_h->m_timeout_event_p->remove_dynamic(method_h);
	    method_h->m_timeout_event_p->cancel();
	}
        if ( m_event_p ) m_event_p->remove_dynamic( method_h );
        if ( m_event_list_p )
        {
            m_event_list_p->remove_dynamic( method_h, 0 );
            m_event_list_p->auto_delete();
	    m_event_list_p = 0;
        }
        break;
      default: // Some other type, it needs to clean up itself.
        // std::cout << "Check " << __FILE__ << ":" << __LINE__ << std::endl;
        break;
    }
}

//------------------------------------------------------------------------------
//"sc_process_b::remove_static_events"
//
// This method removes this object instance from the events in its static
// event list.
//------------------------------------------------------------------------------
void
sc_process_b::remove_static_events()
{
    sc_method_handle method_h; // This process as a method.
    sc_thread_handle thread_h; // This process as a thread.

    switch ( m_process_kind )
    {
      case SC_THREAD_PROC_:
      case SC_CTHREAD_PROC_:
        thread_h = SCAST<sc_thread_handle>( this );
        for( int i = m_static_events.size() - 1; i >= 0; -- i ) {
            m_static_events[i]->remove_static( thread_h );
        }
        m_static_events.resize(0);
        break;
      case SC_METHOD_PROC_:
        method_h = DCAST<sc_method_handle>( this );
        assert( method_h != 0 );
        for( int i = m_static_events.size() - 1; i >= 0; -- i ) {
            m_static_events[i]->remove_static( method_h );
        }
        m_static_events.resize(0);
        break;
      default: // Some other type, it needs to clean up itself.
        // std::cout << "Check " << __FILE__ << ":" << __LINE__ << std::endl;
        break;
    }
}

//------------------------------------------------------------------------------
// "sc_process_b::report_error"
//
// This method can be used to issue a report from within a process.
// The error of the given ID is reported with the given message and
// the process' name() appended to the report.
//------------------------------------------------------------------------------
void
sc_process_b::report_error( const char* msgid, const char* msg ) const
{
    std::stringstream sstr;
    if( msg && msg[0] )
        sstr << msg << ": ";
    sstr << name();
    SC_REPORT_ERROR( msgid, sstr.str().c_str() );
}


//------------------------------------------------------------------------------
// "sc_process_b::report_immediate_self_notification"
//
// This method is used to report an immediate self-notification
// that used to trigger the process before the clarification in 1666-2011.
// The warning is only reported once.
//------------------------------------------------------------------------------
void
sc_process_b::report_immediate_self_notification() const
{
    static bool once = false;
    if( !once ) {
      SC_REPORT_WARNING( SC_ID_IMMEDIATE_SELF_NOTIFICATION_, name() );
      once = true;
    }
}

//------------------------------------------------------------------------------
//"sc_process_b::reset_changed"
//      
// This method is called when there is a change in the value of the
// signal that was specified via reset_signal_is, or the value of the
// m_sticky_reset field. We get called any time m_sticky_reset changes
// or a signal value changes since, since we may need to throw an exception 
// or clear one. Note that this method may be called when there is no
// active process, but rather the main simulator is executing so we must
// check for that case.
//
// Arguments:
//     async    = true if this is an asynchronous reset.
//     asserted = true if reset being asserted, false if being deasserted.
//------------------------------------------------------------------------------
void sc_process_b::reset_changed( bool async, bool asserted )
{       

    // Error out on the corner case:

    if ( !sc_allow_process_control_corners && !async && 
         (m_state & ps_bit_suspended) )
    {
	report_error( SC_ID_PROCESS_CONTROL_CORNER_CASE_,
	              "synchronous reset changed on a suspended process" );
    }

    // IF THIS OBJECT IS PUSHING UP DAISIES WE ARE DONE:

    if ( m_state & ps_bit_zombie ) return;    

    // Reset is being asserted:

    if ( asserted )
    {
        // if ( m_reset_event_p ) m_reset_event_p->notify();
        if ( async )
	{
	    m_active_areset_n++;
	    if ( sc_is_running() ) throw_reset(true);
	}
	else
	{
	    m_active_reset_n++;
	    if ( sc_is_running() ) throw_reset(false);
	}
    }

    // Reset is being deasserted:

    else 
    {
        if ( async )
	{
	    m_active_areset_n--;
	}
	else
	{
	    m_active_reset_n--;
	}
    }

    // Clear the throw type if there are no active resets.

    if ( (m_throw_status == THROW_SYNC_RESET || 
          m_throw_status == THROW_ASYNC_RESET) &&
         m_active_areset_n == 0 && m_active_reset_n == 0 && !m_sticky_reset )
    {
        m_throw_status = THROW_NONE;
    }
}       

//------------------------------------------------------------------------------
//"sc_process_b::reset_event"
//
// This method returns a reference to the reset event for this object 
// instance. If no event exists one is allocated.
//------------------------------------------------------------------------------
sc_event& sc_process_b::reset_event()
{
    if ( !m_reset_event_p ) 
    {
        m_reset_event_p = new sc_event(
	         (std::string(SC_KERNEL_EVENT_PREFIX)+"_reset_event").c_str() );
    }
    return *m_reset_event_p;
}

//------------------------------------------------------------------------------
//"sc_process_b::reset_process"
//
// This inline method changes the reset state of this object instance and
// conditionally its descendants. 
//
// Notes: 
//   (1) It is called for sync_reset_on() and sync_reset_off(). It is not used 
//       for signal sensitive resets, though all reset flow ends up in
//       reset_changed().
//
// Arguments:
//     rt = source of the reset:
//            * reset_asynchronous     - sc_process_handle::reset()
//            * reset_synchronous_off  - sc_process_handle::sync_reset_off()
//            * reset_synchronous_on   - sc_process_handle::sync_reset_on()
//     descendants = indication of how to process descendants.
//------------------------------------------------------------------------------
void sc_process_b::reset_process( reset_type rt,
                                  sc_descendant_inclusion_info descendants )
{

    // PROCESS THIS OBJECT INSTANCE'S DESCENDANTS IF REQUESTED TO:

    if ( descendants == SC_INCLUDE_DESCENDANTS )
    {
        const std::vector<sc_object*> children = get_child_objects();
        int                           child_n  = children.size();

        for ( int child_i = 0; child_i < child_n; child_i++ )
        {
            sc_process_b* child_p = DCAST<sc_process_b*>(children[child_i]);
            if ( child_p ) child_p->reset_process(rt, descendants);
        }
    }

    // PROCESS THIS OBJECT INSTANCE:

    switch (rt)
    {
      // One-shot asynchronous reset: remove dynamic sensitivity and throw:
      //
      // If this is an sc_method only throw if it is active.

      case reset_asynchronous:
	if ( sc_get_status() != SC_RUNNING )
	{
	    report_error(SC_ID_RESET_PROCESS_WHILE_NOT_RUNNING_);
	}
	else
	{
	    remove_dynamic_events();
	    throw_reset(true);
	}
        break;

      // Turn on sticky synchronous reset: use standard reset mechanism.

      case reset_synchronous_on:
	if ( m_sticky_reset == false )
	{
	    m_sticky_reset = true;
	    reset_changed( false, true );
	}
        break;

      // Turn off sticky synchronous reset: use standard reset mechanism.

      default:
	if ( m_sticky_reset == true )
	{
	    m_sticky_reset = false;
	    reset_changed( false, false );
	}
        break;
    }   
}

//------------------------------------------------------------------------------
//"sc_process_b::sc_process_b"
//
// This is the object instance constructor for this class.
//------------------------------------------------------------------------------
sc_process_b::sc_process_b( const char* name_p, bool is_thread, bool free_host,
     SC_ENTRY_FUNC method_p, sc_process_host* host_p, 
     const sc_spawn_options* /* opt_p  */
) :
    sc_object( name_p ),
    file(0),
    lineno(0),
    proc_id( simcontext()->next_proc_id()),
    m_active_areset_n(0),
    m_active_reset_n(0),
    m_dont_init( false ),
    m_dynamic_proc( simcontext()->elaboration_done() ),
    m_event_p(0),
    m_event_count(0),
    m_event_list_p(0),
    m_exist_p(0),
    m_free_host( free_host ),
    m_has_reset_signal( false ),
    m_has_stack(false),
    m_is_thread(is_thread),
    m_last_report_p(0),
    m_name_gen_p(0),
    m_process_kind(SC_NO_PROC_),
    m_references_n(1), 
    m_resets(),
    m_reset_event_p(0),
    m_resume_event_p(0),
    m_runnable_p(0),
    m_semantics_host_p( host_p ),
    m_semantics_method_p ( method_p ),
    m_state(ps_normal),
    m_static_events(),
    m_sticky_reset(false),
    m_term_event_p(0),
    m_throw_helper_p(0),
    m_throw_status( THROW_NONE ),
    m_timed_out(false),
    m_timeout_event_p(0),
    m_trigger_type(STATIC),
    m_unwinding(false)
{

    // THIS OBJECT INSTANCE IS NOW THE LAST CREATED PROCESS:

    m_last_created_process_p = this;
    m_timeout_event_p = new sc_event(
	          (std::string(SC_KERNEL_EVENT_PREFIX)+"_free_event").c_str() );
}

//------------------------------------------------------------------------------
//"sc_process_b::~sc_process_b"
//
// This is the object instance destructor for this class.
//------------------------------------------------------------------------------
sc_process_b::~sc_process_b()
{
   
    // REDIRECT ANY CHILDREN AS CHILDREN OF THE SIMULATION CONTEXT:

    orphan_child_objects();


    // DELETE SEMANTICS OBJECTS IF NEED BE:

    if ( m_free_host ) delete m_semantics_host_p;
#   if !defined(SC_USE_MEMBER_FUNC_PTR) // Remove invocation object.
        delete m_semantics_method_p;
#   endif


    // REMOVE ANY STRUCTURES THAT MAY HAVE BEEN BUILT:

    delete m_last_report_p;
    delete m_name_gen_p;
    delete m_reset_event_p;
    delete m_resume_event_p;
    delete m_term_event_p;
    delete m_throw_helper_p;
    delete m_timeout_event_p;

}

//------------------------------------------------------------------------------
//"sc_process_b::terminated_event"
//
// This method returns a reference to the terminated event for this object 
// instance. If no event exists one is allocated.
//------------------------------------------------------------------------------
sc_event& sc_process_b::terminated_event()
{
    if ( !m_term_event_p ) 
    {
        m_term_event_p = new sc_event(
	          (std::string(SC_KERNEL_EVENT_PREFIX)+"_term_event").c_str() );
    }
    return *m_term_event_p;
}

// +----------------------------------------------------------------------------
// |"sc_process_b::trigger_reset_event"
// | 
// | This method triggers the notify event. It exists because we can't get
// | sc_event context in sc_process.h because the includes would be 
// | circular... sigh...
// +----------------------------------------------------------------------------
void sc_process_b::trigger_reset_event()
{
    if ( m_reset_event_p ) m_reset_event_p->notify();
}

//------------------------------------------------------------------------------
//"sc_process_handle::operator sc_cthread_handle"
//
//------------------------------------------------------------------------------
sc_process_handle::operator sc_cthread_handle()  
{
    return DCAST<sc_cthread_handle>(m_target_p); 
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_method_handle"
//
//------------------------------------------------------------------------------
sc_process_handle::operator sc_method_handle()  
{
    return DCAST<sc_method_handle>(m_target_p); 
}

//------------------------------------------------------------------------------
//"sc_process_handle::sc_thread_handle"
//
//------------------------------------------------------------------------------
sc_process_handle::operator sc_thread_handle()  
{
    return DCAST<sc_thread_handle>(m_target_p); 
}

} // namespace sc_core 


/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte Design Systems, 12 Aug 05
  Description of Modification: This is the rewrite of process support. It 
                               contains some code from the now-defunct 
                               sc_process_b.cpp, as well as the former 
                               version of sc_process_b.cpp.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_process.cpp,v $
// Revision 1.37  2011/08/24 22:05:51  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.36  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
//
// Revision 1.35  2011/08/07 19:08:04  acg
//  Andy Goodrich: moved logs to end of file so line number synching works
//  better between versions.
//
// Revision 1.34  2011/07/29 22:55:01  acg
//  Philipp A. Hartmann: add missing include.
//
// Revision 1.33  2011/07/29 22:43:41  acg
//   Philipp A. Hartmann: changes to handle case where a process control
//   invocation on a child process causes the list of child processes to change.
//
// Revision 1.32  2011/07/24 11:20:03  acg
//  Philipp A. Hartmann: process control error message improvements:
//  (1) Downgrade error to warning for re-kills of processes.
//  (2) Add process name to process messages.
//  (3) drop some superfluous colons in messages.
//
// Revision 1.31  2011/04/19 15:04:27  acg
//  Philipp A. Hartmann: clean up SC_ID messages.
//
// Revision 1.30  2011/04/14 22:33:43  acg
//  Andy Goodrich: added missing checks for a process being a zombie.
//
// Revision 1.29  2011/04/13 05:00:43  acg
//  Andy Goodrich: removed check for method process in termination_event()
//  since with the new IEEE 1666 2011 its legal.
//
// Revision 1.28  2011/04/13 02:44:26  acg
//  Andy Goodrich: added m_unwinding flag in place of THROW_NOW because the
//  throw status will be set back to THROW_*_RESET if reset is active and
//  the check for an unwind being complete was expecting THROW_NONE as the
//  clearing of THROW_NOW.
//
// Revision 1.27  2011/04/10 22:17:35  acg
//  Andy Goodrich: added trigger_reset_event() to allow sc_process.h to
//  contain the run_process() inline method. sc_process.h cannot have
//  sc_simcontext information because of recursive includes.
//
// Revision 1.26  2011/04/08 22:33:08  acg
//  Andy Goodrich: moved the semantics() method to the header file and made
//  it an inline method.
//
// Revision 1.25  2011/04/08 18:24:48  acg
//  Andy Goodrich: moved reset_changed() to .cpp since it needs visibility
//  to sc_simcontext.
//
// Revision 1.24  2011/04/05 20:50:57  acg
//  Andy Goodrich:
//    (1) changes to make sure that event(), posedge() and negedge() only
//        return true if the clock has not moved.
//    (2) fixes for method self-resumes.
//    (3) added SC_PRERELEASE_VERSION
//    (4) removed kernel events from the object hierarchy, added
//        sc_hierarchy_name_exists().
//
// Revision 1.23  2011/04/05 06:25:38  acg
//  Andy Goodrich: new checks for simulation running in reset_process().
//
// Revision 1.22  2011/03/20 13:43:23  acg
//  Andy Goodrich: added async_signal_is() plus suspend() as a corner case.
//
// Revision 1.21  2011/03/12 21:07:51  acg
//  Andy Goodrich: changes to kernel generated event support.
//
// Revision 1.20  2011/03/07 17:38:43  acg
//  Andy Goodrich: tightening up of checks for undefined interaction between
//  synchronous reset and suspend.
//
// Revision 1.19  2011/03/06 23:30:13  acg
//  Andy Goodrich: refining suspend - sync reset corner case checking so that
//  the following are error situations:
//    (1) Calling suspend on a process with a reset_signal_is() specification
//        or sync_reset_on() is active.
//    (2) Calling sync_reset_on() on a suspended process.
//
// Revision 1.18  2011/03/06 19:57:11  acg
//  Andy Goodrich: refinements for the illegal suspend - synchronous reset
//  interaction.
//
// Revision 1.17  2011/03/06 16:47:09  acg
//  Andy Goodrich: changes for testing sync_reset - suspend corner cases.
//
// Revision 1.16  2011/03/06 15:57:57  acg
//  Andy Goodrich: added process control corner case checks. Changes for
//  named events.
//
// Revision 1.15  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.14  2011/02/17 19:52:13  acg
//  Andy Goodrich:
//    (1) Simplfied process control usage.
//    (2) Changed dump_status() to dump_state with new signature.
//
// Revision 1.13  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.12  2011/02/13 21:41:34  acg
//  Andy Goodrich: get the log messages for the previous check in correct.
//
// Revision 1.11  2011/02/13 21:32:24  acg
//  Andy Goodrich: moved sc_process_b::reset_process() from header file
//  to cpp file. Added dump_status() to print out the status of a
//  process.
//
// Revision 1.10  2011/02/04 15:27:36  acg
//  Andy Goodrich: changes for suspend-resume semantics.
//
// Revision 1.9  2011/02/01 21:06:12  acg
//  Andy Goodrich: new layout for the process_state enum.
//
// Revision 1.8  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.7  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.6  2011/01/18 20:10:45  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.5  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.4  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.3  2008/05/22 17:06:26  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/09/20 20:32:35  acg
//  Andy Goodrich: changes to the semantics of throw_it() to match the
//  specification. A call to throw_it() will immediately suspend the calling
//  thread until all the throwees have executed. At that point the calling
//  thread will be restarted before the execution of any other threads.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.6  2006/04/20 17:08:17  acg
//  Andy Goodrich: 3.0 style process changes.
//
// Revision 1.5  2006/04/11 23:13:21  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.4  2006/01/24 20:49:05  acg
// Andy Goodrich: changes to remove the use of deprecated features within the
// simulator, and to issue warning messages when deprecated features are used.
//
// Revision 1.3  2006/01/13 18:44:30  acg
// Added $Log to record CVS changes into the source.
//
