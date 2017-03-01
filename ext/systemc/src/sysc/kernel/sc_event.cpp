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

  sc_event.cpp --

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

 CHANGE LOG APPEARS AT THE END OF THE FILE
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>

#include "sysc/kernel/sc_event.h"
#include "sysc/kernel/sc_kernel_ids.h"
#include "sysc/kernel/sc_phase_callback_registry.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_process_handle.h"
#include "sysc/kernel/sc_simcontext_int.h"
#include "sysc/kernel/sc_object_manager.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_event
//
//  The event class.
// ----------------------------------------------------------------------------

const char*
sc_event::basename() const
{
    const char* p = strrchr( m_name.c_str(), SC_HIERARCHY_CHAR );
    return p ? (p + 1) : m_name.c_str();
}

void
sc_event::cancel()
{
    // cancel a delta or timed notification
    switch( m_notify_type ) {
    case DELTA: {
        // remove this event from the delta events set
        m_simc->remove_delta_event( this );
        m_notify_type = NONE;
        break;
    }
    case TIMED: {
        // remove this event from the timed events set
        sc_assert( m_timed != 0 );
        m_timed->m_event = 0;
        m_timed = 0;
        m_notify_type = NONE;
        break;
    }
    default:
        ;
    }
}


void
sc_event::notify()
{
    // immediate notification
    if(
        // coming from sc_prim_channel::update
        m_simc->update_phase()
#if SC_HAS_PHASE_CALLBACKS_
        // coming from phase callbacks
        || m_simc->notify_phase()
#endif
      )
    {
        SC_REPORT_ERROR( SC_ID_IMMEDIATE_NOTIFICATION_, "" );
        return;
    }
    cancel();
    trigger();
}

void
sc_event::notify( const sc_time& t )
{
    if( m_notify_type == DELTA ) {
        return;
    }
    if( t == SC_ZERO_TIME ) {
#       if SC_HAS_PHASE_CALLBACKS_
            if( SC_UNLIKELY_( m_simc->get_status()
                              & (SC_END_OF_UPDATE|SC_BEFORE_TIMESTEP) ) )
            {
                std::stringstream msg;
                msg << m_simc->get_status()
                    << ":\n\t delta notification of `"
                    << name() << "' ignored";
                SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_FORBIDDEN_
                                 , msg.str().c_str() );
                return;
            }
#       endif
        if( m_notify_type == TIMED ) {
            // remove this event from the timed events set
            sc_assert( m_timed != 0 );
            m_timed->m_event = 0;
            m_timed = 0;
        }
        // add this event to the delta events set
        m_delta_event_index = m_simc->add_delta_event( this );
        m_notify_type = DELTA;
        return;
    }
#   if SC_HAS_PHASE_CALLBACKS_
        if( SC_UNLIKELY_( m_simc->get_status()
                        & (SC_END_OF_UPDATE|SC_BEFORE_TIMESTEP) ) )
        {
            std::stringstream msg;
            msg << m_simc->get_status()
                << ":\n\t timed notification of `"
                << name() << "' ignored";
            SC_REPORT_WARNING( SC_ID_PHASE_CALLBACK_FORBIDDEN_
                             , msg.str().c_str() );
            return;
        }
#   endif
    if( m_notify_type == TIMED ) {
        sc_assert( m_timed != 0 );
        if( m_timed->m_notify_time <= m_simc->time_stamp() + t ) {
            return;
        }
        // remove this event from the timed events set
        m_timed->m_event = 0;
        m_timed = 0;
    }
    // add this event to the timed events set
    sc_event_timed* et = new sc_event_timed( this, m_simc->time_stamp() + t );
    m_simc->add_timed_event( et );
    m_timed = et;
    m_notify_type = TIMED;
}

static void sc_warn_notify_delayed()
{
    static bool warn_notify_delayed=true;
    if ( warn_notify_delayed )
    {
        warn_notify_delayed = false;
        SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
      "notify_delayed(...) is deprecated, use notify(sc_time) instead" );
    }
}

void
sc_event::notify_delayed()
{
    sc_warn_notify_delayed();
    if( m_notify_type != NONE ) {
        SC_REPORT_ERROR( SC_ID_NOTIFY_DELAYED_, 0 );
    }
    // add this event to the delta events set
    m_delta_event_index = m_simc->add_delta_event( this );
    m_notify_type = DELTA;
}

void
sc_event::notify_delayed( const sc_time& t )
{
    sc_warn_notify_delayed();
    if( m_notify_type != NONE ) {
        SC_REPORT_ERROR( SC_ID_NOTIFY_DELAYED_, 0 );
    }
    if( t == SC_ZERO_TIME ) {
        // add this event to the delta events set
        m_delta_event_index = m_simc->add_delta_event( this );
        m_notify_type = DELTA;
    } else {
        // add this event to the timed events set
        sc_event_timed* et = new sc_event_timed( this,
                                                 m_simc->time_stamp() + t );
        m_simc->add_timed_event( et );
        m_timed = et;
        m_notify_type = TIMED;
    }
}

// +----------------------------------------------------------------------------
// |"sc_event::register_event"
// | 
// | This method sets the name of this object instance and optionally adds 
// | it to the object manager's hierarchy. The object instance will be
// | inserted into the object manager's hierarchy if one of the following is
// | true:
// |   (a) the leaf name is non-null and does not start with  
// |       SC_KERNEL_EVENT_PREFIX.
// |   (b) the event is being created before the start of simulation.
// |
// | Arguments:
// |     leaf_name = leaf name of the object or NULL.
// +----------------------------------------------------------------------------
void sc_event::register_event( const char* leaf_name )
{
    sc_object_manager* object_manager = m_simc->get_object_manager();
    m_parent_p = m_simc->active_object();

    // No name provided, if we are not executing then create a name:

    if( !leaf_name || !leaf_name[0] )
    {
	if ( sc_is_running( m_simc ) ) return;
        leaf_name = sc_gen_unique_name("event");    
    }

    // Create a hierarchichal name and place it into the object manager if
    // its not a kernel event:

    object_manager->create_name( leaf_name ).swap( m_name );

    if ( strncmp( leaf_name, SC_KERNEL_EVENT_PREFIX, 
                  strlen(SC_KERNEL_EVENT_PREFIX) ) )
    {
	object_manager->insert_event(m_name, this);
	if ( m_parent_p )
	    m_parent_p->add_child_event( this );
	else
	    m_simc->add_child_event( this );
    }
}

void
sc_event::reset()
{
    m_notify_type = NONE;
    m_delta_event_index = -1;
    m_timed = 0;
    // clear the dynamic sensitive methods
    m_methods_dynamic.resize(0);
    // clear the dynamic sensitive threads
    m_threads_dynamic.resize(0);
}

// +----------------------------------------------------------------------------
// |"sc_event::sc_event(name)"
// | 
// | This is the object instance constructor for named sc_event instances.
// | If the name is non-null or the this is during elaboration add the
// | event to the object hierarchy.
// |
// | Arguments:
// |     name = name of the event.
// +----------------------------------------------------------------------------
sc_event::sc_event( const char* name ) :
    m_name(),
    m_parent_p(NULL),
    m_simc( sc_get_curr_simcontext() ),
    m_notify_type( NONE ),
    m_delta_event_index( -1 ),
    m_timed( 0 ),
    m_methods_static(),
    m_methods_dynamic(),
    m_threads_static(),
    m_threads_dynamic()
{
    // Skip simulator's internally defined events.

    register_event( name );
}

// +----------------------------------------------------------------------------
// |"sc_event::sc_event(name)"
// | 
// | This is the object instance constructor for non-named sc_event instances.
// | If this is during elaboration add create a name and add it to the object
// | hierarchy.
// +----------------------------------------------------------------------------
sc_event::sc_event() :
    m_name(),
    m_parent_p(NULL),
    m_simc( sc_get_curr_simcontext() ),
    m_notify_type( NONE ),
    m_delta_event_index( -1 ),
    m_timed( 0 ),
    m_methods_static(),
    m_methods_dynamic(),
    m_threads_static(),
    m_threads_dynamic()
{

    register_event( NULL );
}

// +----------------------------------------------------------------------------
// |"sc_event::~sc_event"
// | 
// | This is the object instance destructor for this class. It cancels any
// | outstanding waits and removes the event from the object manager's 
// | instance table if it has a name.
// +----------------------------------------------------------------------------
sc_event::~sc_event()
{
    cancel();
    if ( m_name.length() != 0 )
    {
	sc_object_manager* object_manager_p = m_simc->get_object_manager();
	object_manager_p->remove_event( m_name );
    }
}

// +----------------------------------------------------------------------------
// |"sc_event::trigger"
// | 
// | This method "triggers" this object instance. This consists of scheduling
// | for execution all the processes that are schedulable and waiting on this 
// | event.
// +----------------------------------------------------------------------------
void
sc_event::trigger()
{
    int       last_i; // index of last element in vector now accessing.
    int       size;   // size of vector now accessing.


    // trigger the static sensitive methods

    if( ( size = m_methods_static.size() ) != 0 ) 
    {
        sc_method_handle* l_methods_static = &m_methods_static[0];
        int i = size - 1;
        do {
            sc_method_handle method_h = l_methods_static[i];
            method_h->trigger_static();
        } while( -- i >= 0 );
    }

    // trigger the dynamic sensitive methods


    if( ( size = m_methods_dynamic.size() ) != 0 ) 
    {
	last_i = size - 1;
	sc_method_handle* l_methods_dynamic = &m_methods_dynamic[0];
	for ( int i = 0; i <= last_i; i++ )
	{
	    sc_method_handle method_h = l_methods_dynamic[i];
	    if ( method_h->trigger_dynamic( this ) )
	    {
		l_methods_dynamic[i] = l_methods_dynamic[last_i];
		last_i--;
		i--;
	    }
	}
        m_methods_dynamic.resize(last_i+1);
    }


    // trigger the static sensitive threads

    if( ( size = m_threads_static.size() ) != 0 ) 
    {
        sc_thread_handle* l_threads_static = &m_threads_static[0];
        int i = size - 1;
        do {
            sc_thread_handle thread_h = l_threads_static[i];
            thread_h->trigger_static();
        } while( -- i >= 0 );
    }

    // trigger the dynamic sensitive threads

    if( ( size = m_threads_dynamic.size() ) != 0 ) 
    {
	last_i = size - 1;
	sc_thread_handle* l_threads_dynamic = &m_threads_dynamic[0];
	for ( int i = 0; i <= last_i; i++ )
	{
	    sc_thread_handle thread_h = l_threads_dynamic[i];
	    if ( thread_h->trigger_dynamic( this ) )
	    {
		l_threads_dynamic[i] = l_threads_dynamic[last_i];
		i--;
		last_i--;
	    }
	}
        m_threads_dynamic.resize(last_i+1);
    }

    m_notify_type = NONE;
    m_delta_event_index = -1;
    m_timed = 0;
}


bool
sc_event::remove_static( sc_method_handle method_h_ ) const
{
    int size;
    if ( ( size = m_methods_static.size() ) != 0 ) {
      sc_method_handle* l_methods_static = &m_methods_static[0];
      for( int i = size - 1; i >= 0; -- i ) {
          if( l_methods_static[i] == method_h_ ) {
              l_methods_static[i] = l_methods_static[size - 1];
              m_methods_static.resize(size-1);
              return true;
          }
      }
    }
    return false;
}

bool
sc_event::remove_static( sc_thread_handle thread_h_ ) const
{
    int size;
    if ( ( size = m_threads_static.size() ) != 0 ) {
      sc_thread_handle* l_threads_static = &m_threads_static[0];
      for( int i = size - 1; i >= 0; -- i ) {
          if( l_threads_static[i] == thread_h_ ) {
              l_threads_static[i] = l_threads_static[size - 1];
              m_threads_static.resize(size-1);
              return true;
          }
      }
    }
    return false;
}

bool
sc_event::remove_dynamic( sc_method_handle method_h_ ) const
{
    int size;
    if ( ( size = m_methods_dynamic.size() ) != 0 ) {
      sc_method_handle* l_methods_dynamic = &m_methods_dynamic[0];
      for( int i = size - 1; i >= 0; -- i ) {
          if( l_methods_dynamic[i] == method_h_ ) {
              l_methods_dynamic[i] = l_methods_dynamic[size - 1];
              m_methods_dynamic.resize(size-1);
              return true;
          }
      }
    }
    return false;
}

bool
sc_event::remove_dynamic( sc_thread_handle thread_h_ ) const
{
    int size;
    if ( ( size= m_threads_dynamic.size() ) != 0 ) {
      sc_thread_handle* l_threads_dynamic = &m_threads_dynamic[0];
      for( int i = size - 1; i >= 0; -- i ) {
          if( l_threads_dynamic[i] == thread_h_ ) {
              l_threads_dynamic[i] = l_threads_dynamic[size - 1];
              m_threads_dynamic.resize(size-1);
              return true;
          }
      }
    }
    return false;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_event_timed
//
//  Class for storing the time to notify a timed event.
// ----------------------------------------------------------------------------

// dedicated memory management; not MT-Safe

union sc_event_timed_u
{
    sc_event_timed_u* next;
    char              dummy[sizeof( sc_event_timed )];
};

static
sc_event_timed_u* free_list = 0;

void*
sc_event_timed::allocate()
{
    const int ALLOC_SIZE = 64;

    if( free_list == 0 ) {
        free_list = (sc_event_timed_u*) malloc( ALLOC_SIZE *
                                                sizeof( sc_event_timed ) );
        int i = 0;
        for( ; i < ALLOC_SIZE - 1; ++ i ) {
            free_list[i].next = &free_list[i + 1];
        }
        free_list[i].next = 0;
    }

    sc_event_timed_u* q = free_list;
    free_list = free_list->next;
    return q;
}

void
sc_event_timed::deallocate( void* p )
{
    if( p != 0 ) {
        sc_event_timed_u* q = RCAST<sc_event_timed_u*>( p );
        q->next = free_list;
        free_list = q;
    }
}


// ----------------------------------------------------------------------------
//  CLASS : sc_event_list
//
//  Base class for lists of events.
// ----------------------------------------------------------------------------

void
sc_event_list::push_back( const sc_event& e )
{
    // make sure e is not already in the list
    if ( m_events.size() != 0 ) {
      const sc_event** l_events = &m_events[0];
      for( int i = m_events.size() - 1; i >= 0; -- i ) {
          if( &e == l_events[i] ) {
              // event already in the list; ignore
              return;
          }
      }
    }
    m_events.push_back( &e );
}

void 
sc_event_list::push_back( const sc_event_list& el )
{
    m_events.reserve( size() + el.size() );
    for ( int i = el.m_events.size() - 1; i >= 0; --i )
    {
        push_back( *el.m_events[i] );
    }
    el.auto_delete();
}

void
sc_event_list::add_dynamic( sc_method_handle method_h ) const
{
    m_busy++;
    if ( m_events.size() != 0 ) {
      const sc_event* const * l_events = &m_events[0];
      for( int i = m_events.size() - 1; i >= 0; -- i ) {
          l_events[i]->add_dynamic( method_h );
      }
  }
}

void
sc_event_list::add_dynamic( sc_thread_handle thread_h ) const
{
    m_busy++;
    if ( m_events.size() != 0 ) {
      const sc_event* const* l_events = &m_events[0];
      for( int i = m_events.size() - 1; i >= 0; -- i ) {
          l_events[i]->add_dynamic( thread_h );
      }
  }
}

void
sc_event_list::remove_dynamic( sc_method_handle method_h,
                               const sc_event* e_not ) const
{
    if ( m_events.size() != 0 ) {
      const sc_event* const* l_events = &m_events[0];
      for( int i = m_events.size() - 1; i >= 0; -- i ) {
          const sc_event* e = l_events[i];
          if( e != e_not ) {
              e->remove_dynamic( method_h );
          }
      }
  }
}

void
sc_event_list::remove_dynamic( sc_thread_handle thread_h,
                               const sc_event* e_not ) const
{
    if ( m_events.size() != 0 ) {
      const sc_event* const* l_events = &m_events[0];
      for( int i = m_events.size() - 1; i >= 0; -- i ) {
          const sc_event* e = l_events[i];
          if( e != e_not ) {
              e->remove_dynamic( thread_h );
          }
      }
  }
}

void
sc_event_list::report_premature_destruction() const
{
    // TDB: reliably detect premature destruction
    //
    // If an event list is used as a member of a module,
    // its lifetime may (correctly) end, although there
    // are processes currently waiting for it.
    //
    // Detecting (and ignoring) this corner-case is quite
    // difficult for similar reasons to the sc_is_running()
    // return value during the destruction of the module
    // hierarchy.
    //
    // Ignoring the lifetime checks for now, if no process
    // is currently running (which is only part of the story):

    if( sc_get_current_process_handle().valid() ) {
        // FIXME: improve error-handling
        sc_assert( false && "sc_event_list prematurely destroyed" );
    }

}

void
sc_event_list::report_invalid_modification() const
{
    // FIXME: improve error-handling
    sc_assert( false && "sc_event_list modfied while being waited on" );
}

// ----------------------------------------------------------------------------
//  Deprecated functional notation for notifying events.
// ----------------------------------------------------------------------------

static void sc_warn_notify()
{
    static bool warn_notify=true;
    if ( warn_notify )
    {
  SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_,
      "the notify() function is deprecated use sc_event::notify()" );
  warn_notify = false;
    }
}

void
notify( sc_event& e )
{
    sc_warn_notify();
    e.notify();
}

void
notify( const sc_time& t, sc_event& e )
{
    sc_warn_notify();
    e.notify( t );
}

void
notify( double v, sc_time_unit tu, sc_event& e )
{
    sc_warn_notify();
    e.notify( v, tu );
}

} // namespace sc_core

// $Log: sc_event.cpp,v $
// Revision 1.17  2011/08/26 20:46:09  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.16  2011/08/24 22:05:50  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.15  2011/03/12 21:07:51  acg
//  Andy Goodrich: changes to kernel generated event support.
//
// Revision 1.14  2011/03/06 15:55:52  acg
//  Andy Goodrich: changes for named events.
//
// Revision 1.13  2011/03/05 01:39:21  acg
//  Andy Goodrich: changes for named events.
//
// Revision 1.12  2011/02/19 08:33:25  acg
//  Andy Goodrich: remove }'s that should have been removed before.
//
// Revision 1.11  2011/02/19 08:30:53  acg
//  Andy Goodrich: Moved process queueing into trigger_static from
//  sc_event::notify.
//
// Revision 1.10  2011/02/18 20:27:14  acg
//  Andy Goodrich: Updated Copyrights.
//
// Revision 1.9  2011/02/17 19:49:51  acg
//  Andy Goodrich:
//    (1) Changed signature of trigger_dynamic() to return a bool again.
//    (2) Moved process run queue processing into trigger_dynamic().
//
// Revision 1.8  2011/02/16 22:37:30  acg
//  Andy Goodrich: clean up to remove need for ps_disable_pending.
//
// Revision 1.7  2011/02/13 21:47:37  acg
//  Andy Goodrich: update copyright notice.
//
// Revision 1.6  2011/02/01 21:02:28  acg
//  Andy Goodrich: new return code for trigger_dynamic() calls.
//
// Revision 1.5  2011/01/18 20:10:44  acg
//  Andy Goodrich: changes for IEEE1666_2011 semantics.
//
// Revision 1.4  2011/01/06 18:04:05  acg
//  Andy Goodrich: added code to leave disabled processes on the dynamic
//  method and thread queues.
//
// Revision 1.3  2008/05/22 17:06:25  acg
//  Andy Goodrich: updated copyright notice to include 2008.
//
// Revision 1.2  2007/01/17 22:44:30  acg
//  Andy Goodrich: fix for Microsoft compiler.
//
// Revision 1.7  2006/04/11 23:13:20  acg
//   Andy Goodrich: Changes for reduced reset support that only includes
//   sc_cthread, but has preliminary hooks for expanding to method and thread
//   processes also.
//
// Revision 1.6  2006/01/25 00:31:19  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.5  2006/01/24 20:59:11  acg
//  Andy Goodrich: fix up of CVS comments, new version roll.
//
// Revision 1.4  2006/01/24 20:48:14  acg
// Andy Goodrich: added deprecation warnings for notify_delayed(). Added two
// new implementation-dependent methods, notify_next_delta() & notify_internal()
// to replace calls to notify_delayed() from within the simulator. These two
// new methods are simpler than notify_delayed() and should speed up simulations
//
// Revision 1.3  2006/01/13 18:44:29  acg
// Added $Log to record CVS changes into the source.

// Taf!
