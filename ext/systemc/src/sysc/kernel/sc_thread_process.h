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

  sc_thread_process.h -- Thread process declarations

  Original Author: Andy Goodrich, Forte Design Systems, 4 August 2005
               

  CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/


#if !defined(sc_thread_process_h_INCLUDED)
#define sc_thread_process_h_INCLUDED

#include "sysc/kernel/sc_spawn_options.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_cor.h"
#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_except.h"
#include "sysc/kernel/sc_reset.h"

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

// forward references:
class sc_event_and_list;
class sc_event_or_list;
class sc_reset;
void sc_thread_cor_fn( void* );
void sc_set_stack_size( sc_thread_handle, std::size_t );
class sc_event;
class sc_join;
class sc_module;
class sc_process_handle;
class sc_process_table;
class sc_simcontext;
class sc_runnable;

sc_cor* get_cor_pointer( sc_process_b* process_p );
void sc_set_stack_size( sc_thread_handle thread_h, std::size_t size );
void wait( sc_simcontext* );
void wait( const sc_event&, sc_simcontext* );
void wait( const sc_event_or_list&, sc_simcontext* );
void wait( const sc_event_and_list&, sc_simcontext* );
void wait( const sc_time&, sc_simcontext* );
void wait( const sc_time&, const sc_event&, sc_simcontext* );
void wait( const sc_time&, const sc_event_or_list&, sc_simcontext* );
void wait( const sc_time&, const sc_event_and_list&, sc_simcontext* );

//==============================================================================
// sc_thread_process -
//
//==============================================================================
class sc_thread_process : public sc_process_b {
    friend void sc_thread_cor_fn( void* );
    friend void sc_set_stack_size( sc_thread_handle, std::size_t );
    friend class sc_event;
    friend class sc_join;
    friend class sc_module;
    friend class sc_process_b;
    friend class sc_process_handle;
    friend class sc_process_table;
    friend class sc_simcontext;
    friend class sc_runnable;
    friend sc_cor* get_cor_pointer( sc_process_b* process_p );

    friend void wait( sc_simcontext* );
    friend void wait( const sc_event&, sc_simcontext* );
    friend void wait( const sc_event_or_list&, sc_simcontext* );
    friend void wait( const sc_event_and_list&, sc_simcontext* );
    friend void wait( const sc_time&, sc_simcontext* );
    friend void wait( const sc_time&, const sc_event&, sc_simcontext* );
    friend void wait( const sc_time&, const sc_event_or_list&, sc_simcontext* );
    friend void wait( const sc_time&, const sc_event_and_list&, sc_simcontext*);
  public:
    sc_thread_process( const char* name_p, bool free_host, 
        SC_ENTRY_FUNC method_p, sc_process_host* host_p, 
        const sc_spawn_options* opt_p );

    virtual const char* kind() const
        { return "sc_thread_process"; }

  protected:
    // may not be deleted manually (called from sc_process_b)
    virtual ~sc_thread_process();

    virtual void disable_process( 
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );
    virtual void enable_process( 
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );
    virtual void kill_process(
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );
    sc_thread_handle next_exist();
    sc_thread_handle next_runnable();
    virtual void prepare_for_simulation();
    virtual void resume_process( 
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );
    void set_next_exist( sc_thread_handle next_p );
    void set_next_runnable( sc_thread_handle next_p );

    void set_stack_size( std::size_t size );
    inline void suspend_me();
    virtual void suspend_process( 
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );
    virtual void throw_reset( bool async );
    virtual void throw_user( const sc_throw_it_helper& helper,
        sc_descendant_inclusion_info descendants = SC_NO_DESCENDANTS );

    bool trigger_dynamic( sc_event* );
    inline void trigger_static();

    void wait( const sc_event& );
    void wait( const sc_event_or_list& );
    void wait( const sc_event_and_list& );
    void wait( const sc_time& );
    void wait( const sc_time&, const sc_event& );
    void wait( const sc_time&, const sc_event_or_list& );
    void wait( const sc_time&, const sc_event_and_list& );
    void wait_cycles( int n=1 );

  protected:
    void add_monitor( sc_process_monitor* monitor_p );
    void remove_monitor( sc_process_monitor* monitor_p);
    void signal_monitors( int type = 0 );

  protected:
    sc_cor*                          m_cor_p;        // Thread's coroutine.
    std::vector<sc_process_monitor*> m_monitor_q;    // Thread monitors.
    std::size_t                      m_stack_size;   // Thread stack size.
    int                              m_wait_cycle_n; // # of waits to be done.

  private: // disabled
    sc_thread_process( const sc_thread_process& );
    const sc_thread_process& operator = ( const sc_thread_process& );

};

//------------------------------------------------------------------------------
//"sc_thread_process::set_stack_size"
//
//------------------------------------------------------------------------------
inline void sc_thread_process::set_stack_size( std::size_t size )
{
    assert( size );
    m_stack_size = size;
}

//------------------------------------------------------------------------------
//"sc_thread_process::suspend_me"
//
// This method suspends this object instance in favor of the next runnable
// process. Upon awakening we check to see if an exception should be thrown. 
// There are two types of exceptions that can be thrown, synchronous reset
// and asynchronous reset. At a future time there may be more asynchronous
// exceptions.  If an asynchronous reset is seen and there is not static reset
// specified, or the static reset is not active then clear the throw
// type for the next time this method is called.
//
// Notes:
//   (1) For an explanation of how the reset mechanism works see the top of
//       the file sc_reset.cpp.
//   (2) The m_sticky_reset field is used to handle synchronous resets that
//       are enabled via the sc_process_handle::sync_reset_on() method. These
//       resets are not generated by a signal, but rather are modal by 
//       method call: sync_reset_on() - sync_reset_off().
//------------------------------------------------------------------------------
inline void sc_thread_process::suspend_me()
{
    // remember, if we're currently unwinding

    bool unwinding_preempted = m_unwinding;

    sc_simcontext* simc_p = simcontext();
    sc_cor*         cor_p = simc_p->next_cor();

    // do not switch, if we're about to execute next (e.g. suicide)

    if( m_cor_p != cor_p )
    {
        DEBUG_MSG( DEBUG_NAME , this, "suspending thread");
        simc_p->cor_pkg()->yield( cor_p );
        DEBUG_MSG( DEBUG_NAME , this, "resuming thread");
    }

    // IF THERE IS A THROW TO BE DONE FOR THIS PROCESS DO IT NOW:
    //
    // (1) Optimize THROW_NONE for speed as it is the normal case.
    // (2) If this thread is already unwinding then suspend_me() was
    //     called from the catch clause to throw an exception on another
    //     process, so just go back to the catch clause.

    if ( m_throw_status == THROW_NONE ) return;

    if ( m_unwinding ) return;

    switch( m_throw_status )
    {
      case THROW_ASYNC_RESET:
      case THROW_SYNC_RESET:
        DEBUG_MSG( DEBUG_NAME , this, "throwing reset for");
	if ( m_reset_event_p ) m_reset_event_p->notify();
        throw sc_unwind_exception( this, true ); 

      case THROW_USER:
        DEBUG_MSG( DEBUG_NAME, this, "invoking throw_it for");
	m_throw_status = m_active_areset_n ? THROW_ASYNC_RESET :
	                                  (m_active_reset_n ? THROW_SYNC_RESET :
			                  THROW_NONE);
        m_throw_helper_p->throw_it();
	break;

      case THROW_KILL:
        DEBUG_MSG( DEBUG_NAME, this, "throwing kill for");
	throw sc_unwind_exception( this, false );

      default: // THROWING_NOW
        sc_assert( unwinding_preempted );
        DEBUG_MSG( DEBUG_NAME, this, "restarting thread");
        break;
    }
}


//------------------------------------------------------------------------------
//"sc_thread_process::wait"
//
//------------------------------------------------------------------------------
inline
void
sc_thread_process::wait( const sc_event& e )
{   
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_event_p = &e; // for cleanup.
    e.add_dynamic( this );
    m_trigger_type = EVENT;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_event_or_list& el )
{   
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    el.add_dynamic( this );
    m_event_list_p = &el;
    m_trigger_type = OR_LIST;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_event_and_list& el )
{
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    el.add_dynamic( this );
    m_event_list_p = &el;
    m_event_count = el.size();
    m_trigger_type = AND_LIST;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_time& t )
{
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_timeout_event_p->notify_internal( t );
    m_timeout_event_p->add_dynamic( this );
    m_trigger_type = TIMEOUT;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_time& t, const sc_event& e )
{
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_timeout_event_p->notify_internal( t );
    m_timeout_event_p->add_dynamic( this );
    e.add_dynamic( this );
    m_event_p = &e;
    m_trigger_type = EVENT_TIMEOUT;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_time& t, const sc_event_or_list& el )
{
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_timeout_event_p->notify_internal( t );
    m_timeout_event_p->add_dynamic( this );
    el.add_dynamic( this );
    m_event_list_p = &el;
    m_trigger_type = OR_LIST_TIMEOUT;
    suspend_me();
}

inline
void
sc_thread_process::wait( const sc_time& t, const sc_event_and_list& el )
{
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_timeout_event_p->notify_internal( t );
    m_timeout_event_p->add_dynamic( this );
    el.add_dynamic( this );
    m_event_list_p = &el;
    m_event_count = el.size();
    m_trigger_type = AND_LIST_TIMEOUT;
    suspend_me();
}

//------------------------------------------------------------------------------
//"sc_thread_process::wait_cycles"
//
// This method suspends this object instance for the specified number of cycles.
// A cycle is defined as the event the thread is set up to staticly wait on.
// The field m_wait_cycle_n is set to one less than the number of cycles to
// be waited for, since the value is tested before being decremented in 
// the simulation kernel.
//------------------------------------------------------------------------------
inline
void
sc_thread_process::wait_cycles( int n )
{   
    if( m_unwinding )
        SC_REPORT_ERROR( SC_ID_WAIT_DURING_UNWINDING_, name() );

    m_wait_cycle_n = n-1;
    suspend_me();
}

//------------------------------------------------------------------------------
//"sc_thread_process::miscellaneous support"
//
//------------------------------------------------------------------------------
inline
void sc_thread_process::add_monitor(sc_process_monitor* monitor_p)
{
    m_monitor_q.push_back(monitor_p);
}


inline
void sc_thread_process::remove_monitor(sc_process_monitor* monitor_p)
{
    int mon_n = m_monitor_q.size();

    for ( int mon_i = 0; mon_i < mon_n; mon_i++ )
    {
    if  ( m_monitor_q[mon_i] == monitor_p )
        {
            m_monitor_q[mon_i] = m_monitor_q[mon_n-1];
            m_monitor_q.resize(mon_n-1);
        }
    }
}

inline
void sc_thread_process::set_next_exist(sc_thread_handle next_p)
{
    m_exist_p = next_p;
}

inline
sc_thread_handle sc_thread_process::next_exist()
{
    return (sc_thread_handle)m_exist_p;
}

inline
void sc_thread_process::set_next_runnable(sc_thread_handle next_p)
{
    m_runnable_p = next_p;
}

inline
sc_thread_handle sc_thread_process::next_runnable()
{
    return (sc_thread_handle)m_runnable_p;
}

inline sc_cor* get_cor_pointer( sc_process_b* process_p )
{
    sc_thread_handle thread_p = DCAST<sc_thread_handle>(process_p);
    return thread_p->m_cor_p;
}

//------------------------------------------------------------------------------
//"sc_thread_process::trigger_static"
//
// This inline method adds the current thread to the queue of runnable
// processes, if required.  This is the case if the following criteria
// are met:
//   (1) The process is in a runnable state.
//   (2) The process is not already on the run queue.
//   (3) The process is expecting a static trigger,
//       dynamic event waits take priority.
//   (4) The process' static wait count is zero.
//
// If the triggering process is the same process, the trigger is
// ignored as well, unless SC_ENABLE_IMMEDIATE_SELF_NOTIFICATIONS
// is defined.
//------------------------------------------------------------------------------
inline
void
sc_thread_process::trigger_static()
{
    // No need to try queueing this thread if one of the following is true:
    //    (a) its disabled
    //    (b) its already queued for execution
    //    (c) its waiting on a dynamic event
    //    (d) its wait count is not satisfied

    if ( (m_state & ps_bit_disabled) || is_runnable() ||
          m_trigger_type != STATIC )
        return;

#if ! defined( SC_ENABLE_IMMEDIATE_SELF_NOTIFICATIONS )
    if( SC_UNLIKELY_( sc_get_current_process_b() == this ) )
    {
        report_immediate_self_notification();
        return;
    }
#endif // SC_ENABLE_IMMEDIATE_SELF_NOTIFICATIONS

    if ( m_wait_cycle_n > 0 )
    {
        --m_wait_cycle_n;
        return;
    }

    // If we get here then the thread is has satisfied its wait criteria, if 
    // its suspended mark its state as ready to run. If its not suspended then 
    // push it onto the runnable queue.

    if ( m_state & ps_bit_suspended )
    {
        m_state = m_state | ps_bit_ready_to_run;
    }
    else
    {
	simcontext()->push_runnable_thread(this);
    }
}

#undef DEBUG_MSG
#undef DEBUG_NAME

} // namespace sc_core 

// $Log: sc_thread_process.h,v $
// Revision 1.30  2011/08/26 20:46:11  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.29  2011/08/24 23:36:12  acg
//  Andy Goodrich: removed break statements that can never be reached and
//  which causes warnings in the Greenhills C++ compiler.
//
// Revision 1.28  2011/04/14 22:34:27  acg
//  Andy Goodrich: removed dead code.
//
// Revision 1.27  2011/04/13 05:02:18  acg
//  Andy Goodrich: added missing check to the wake up code in suspend_me()
//  so that we just return if the call to suspend_me() was issued from a
//  stack unwinding.
//
// Revision 1.26  2011/04/13 02:44:26  acg
//  Andy Goodrich: added m_unwinding flag in place of THROW_NOW because the
//  throw status will be set back to THROW_*_RESET if reset is active and
//  the check for an unwind being complete was expecting THROW_NONE as the
//  clearing of THROW_NOW.
//
// Revision 1.25  2011/04/11 22:05:14  acg
//  Andy Goodrich: use the DEBUG_NAME macro in DEBUG_MSG invocations.
//
// Revision 1.24  2011/04/10 22:12:32  acg
//  Andy Goodrich: adding debugging macros.
//
// Revision 1.23  2011/04/08 22:41:28  acg
//  Andy Goodrich: added comment pointing to the description of the reset
//  mechanism in sc_reset.cpp.
//
// Revision 1.22  2011/04/08 18:27:33  acg
//  Andy Goodrich: added check to make sure we don't schedule a running process
//  because of it issues a notify() it is sensitive to.
//
// Revision 1.21  2011/04/05 06:22:38  acg
//  Andy Goodrich: expanded comment for trigger_static() initial vetting.
//
// Revision 1.20  2011/04/01 21:24:57  acg
//  Andy Goodrich: removed unused code.
//
// Revision 1.19  2011/02/19 08:30:53  acg
//  Andy Goodrich: Moved process queueing into trigger_static from
//  sc_event::notify.
//
// Revision 1.18  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.17  2011/02/17 19:55:58  acg
//  Andy Goodrich:
//    (1) Changed signature of trigger_dynamic() back to a bool.
//    (2) Simplified process control usage.
//    (3) Changed trigger_static() to recognize process controls and to
//        do the down-count on wait(N), allowing the elimination of
//        ready_to_run().
//
// Revision 1.16  2011/02/16 22:37:31  acg
//  Andy Goodrich: clean up to remove need for ps_disable_pending.
//
// Revision 1.15  2011/02/13 21:47:38  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.14  2011/02/13 21:35:54  acg
//  Andy Goodrich: added error for performing a wait() during unwinding.
//
// Revision 1.13  2011/02/11 13:25:24  acg
//  Andy Goodrich: Philipp A. Hartmann's changes:
//    (1) Removal of SC_CTHREAD method overloads.
//    (2) New exception processing code.
//
// Revision 1.12  2011/02/01 23:01:53  acg
//  Andy Goodrich: removed dead code.
//
// Revision 1.11  2011/02/01 21:18:01  acg
//  Andy Goodrich:
//  (1) Changes in throw processing for new process control rules.
//  (2) Support of new process_state enum values.
//
// Revision 1.10  2011/01/25 20:50:37  acg
//  Andy Goodrich: changes for IEEE 1666 2011.
//
// Revision 1.9  2011/01/19 23:21:50  acg
//  Andy Goodrich: changes for IEEE 1666 2011
//
// Revision 1.8  2011/01/18 20:10:45  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.7  2011/01/06 17:59:58  acg
//  Andy Goodrich: removed debugging output.
//
// Revision 1.6  2010/07/22 20:02:33  acg
//  Andy Goodrich: bug fixes.
//
// Revision 1.5  2009/07/28 01:10:53  acg
//  Andy Goodrich: updates for 2.3 release candidate.
//
// Revision 1.4  2009/05/22 16:06:29  acg
//  Andy Goodrich: process control updates.
//
// Revision 1.3  2009/03/12 22:59:58  acg
//  Andy Goodrich: updates for 2.4 stuff.
//
// Revision 1.2  2008/05/22 17:06:06  acg
//  Andy Goodrich: formatting and comments.
//
// Revision 1.1.1.1  2006/12/15 20:20:05  acg
// SystemC 2.3
//
// Revision 1.7  2006/05/08 17:57:13  acg
//  Andy Goodrich: Added David Long's forward declarations for friend functions
//  to keep the Microsoft C++ compiler happy.
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

#endif // !defined(sc_thread_process_h_INCLUDED)
