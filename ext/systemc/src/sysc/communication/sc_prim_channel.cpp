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

  sc_prim_channel.cpp -- Abstract base class of all primitive channel
                         classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#include "sysc/communication/sc_prim_channel.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_module.h"
#include "sysc/kernel/sc_object_int.h"

#ifndef SC_DISABLE_ASYNC_UPDATES
#  include "sysc/communication/sc_host_mutex.h"
#endif

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel
//
//  Abstract base class of all primitive channel classes.
// ----------------------------------------------------------------------------

// constructors

sc_prim_channel::sc_prim_channel()
: sc_object( 0 ),
  m_registry( simcontext()->get_prim_channel_registry() ),
  m_update_next_p( 0 ) 
{
    m_registry->insert( *this );
}

sc_prim_channel::sc_prim_channel( const char* name_ )
: sc_object( name_ ),
  m_registry( simcontext()->get_prim_channel_registry() ),
  m_update_next_p( 0 )
{
    m_registry->insert( *this );
}


// destructor

sc_prim_channel::~sc_prim_channel()
{
    m_registry->remove( *this );
}


// the update method (does nothing by default)

void
sc_prim_channel::update()
{}


// called by construction_done (does nothing by default)

void sc_prim_channel::before_end_of_elaboration() 
{}

// called when construction is done

void
sc_prim_channel::construction_done()
{
    sc_object::hierarchy_scope scope( get_parent_object() );
    before_end_of_elaboration();
}

// called by elaboration_done (does nothing by default)

void
sc_prim_channel::end_of_elaboration()
{}


// called when elaboration is done

void
sc_prim_channel::elaboration_done()
{
    sc_object::hierarchy_scope scope( get_parent_object() );
    end_of_elaboration();
}

// called by start_simulation (does nothing)

void
sc_prim_channel::start_of_simulation()
{}

// called before simulation begins

void
sc_prim_channel::start_simulation()
{
    sc_object::hierarchy_scope scope( get_parent_object() );
    start_of_simulation();
}

// called by simulation_done (does nothing)

void
sc_prim_channel::end_of_simulation()
{}

// called after simulation ends

void
sc_prim_channel::simulation_done()
{
    sc_object::hierarchy_scope scope( get_parent_object() );
    end_of_simulation();
}

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel_registry::async_update_list
//
//  Thread-safe list of pending external updates
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

class sc_prim_channel_registry::async_update_list
{
#ifndef SC_DISABLE_ASYNC_UPDATES
public:

    bool pending() const
    {
	return m_push_queue.size() != 0;
    }

    void append( sc_prim_channel& prim_channel_ )
    {
	sc_scoped_lock lock( m_mutex );
	m_push_queue.push_back( &prim_channel_ );
	// return releases the mutex
    }

    void accept_updates()
    {
	sc_assert( ! m_pop_queue.size() );
	{
	    sc_scoped_lock lock( m_mutex );
	    m_push_queue.swap( m_pop_queue );
	    // leaving the block releases the mutex
	}

	std::vector< sc_prim_channel* >::const_iterator
	    it = m_pop_queue.begin(), end = m_pop_queue.end();
	while( it!= end )
	{
	    // we use request_update instead of perform_update
	    // to skip duplicates
	    (*it++)->request_update();
	}
	m_pop_queue.clear();
    }

private:
    sc_host_mutex                   m_mutex;
    std::vector< sc_prim_channel* > m_push_queue;
    std::vector< sc_prim_channel* > m_pop_queue;

#endif // ! SC_DISABLE_ASYNC_UPDATES
};

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel_registry
//
//  Registry for all primitive channels.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

void
sc_prim_channel_registry::insert( sc_prim_channel& prim_channel_ )
{
    if( sc_is_running() ) {
       SC_REPORT_ERROR( SC_ID_INSERT_PRIM_CHANNEL_, "simulation running" );
    }

    if( m_simc->elaboration_done() ) {

	SC_REPORT_ERROR( SC_ID_INSERT_PRIM_CHANNEL_, "elaboration done" );
    }

#ifdef DEBUG_SYSTEMC
    // check if prim_channel_ is already inserted
    for( int i = 0; i < size(); ++ i ) {
	if( &prim_channel_ == m_prim_channel_vec[i] ) {
	    SC_REPORT_ERROR( SC_ID_INSERT_PRIM_CHANNEL_, "already inserted" );
	}
    }
#endif

    // insert
    m_prim_channel_vec.push_back( &prim_channel_ );

}

void
sc_prim_channel_registry::remove( sc_prim_channel& prim_channel_ )
{
    int i;
    for( i = 0; i < size(); ++ i ) {
	if( &prim_channel_ == m_prim_channel_vec[i] ) {
	    break;
	}
    }
    if( i == size() ) {
	SC_REPORT_ERROR( SC_ID_REMOVE_PRIM_CHANNEL_, 0 );
    }

    // remove
    m_prim_channel_vec[i] = m_prim_channel_vec[size() - 1];
    m_prim_channel_vec.resize(size()-1);
}

bool
sc_prim_channel_registry::pending_async_updates() const
{
#ifndef SC_DISABLE_ASYNC_UPDATES
    return m_async_update_list_p->pending();
#else
    return false;
#endif
}

void
sc_prim_channel_registry::async_request_update( sc_prim_channel& prim_channel_ )
{
#ifndef SC_DISABLE_ASYNC_UPDATES
    m_async_update_list_p->append( prim_channel_ );
#else
    SC_REPORT_ERROR( SC_ID_NO_ASYNC_UPDATE_, prim_channel_.name() );
#endif
}

// +----------------------------------------------------------------------------
// |"sc_prim_channel_registry::perform_update"
// |
// | This method updates the values of the primitive channels in its update
// | lists.
// +----------------------------------------------------------------------------
void
sc_prim_channel_registry::perform_update()
{
    // Update the values for the primitive channels set external to the
    // simulator.

#ifndef SC_DISABLE_ASYNC_UPDATES
    if( m_async_update_list_p->pending() )
	m_async_update_list_p->accept_updates();
#endif

    sc_prim_channel* next_p; // Next update to perform.
    sc_prim_channel* now_p;  // Update now performing.

    // Update the values for the primitive channels in the simulator's list.

    now_p = m_update_list_p;
    m_update_list_p = (sc_prim_channel*)sc_prim_channel::list_end;
    for ( ; now_p != (sc_prim_channel*)sc_prim_channel::list_end;
	now_p = next_p )
    {
	next_p = now_p->m_update_next_p;
	now_p->perform_update();
    }
}

// constructor

sc_prim_channel_registry::sc_prim_channel_registry( sc_simcontext& simc_ )
  :  m_async_update_list_p(0)
  ,  m_construction_done(0)
  ,  m_prim_channel_vec()
  ,  m_simc( &simc_ )
  ,  m_update_list_p((sc_prim_channel*)sc_prim_channel::list_end)
{
#   ifndef SC_DISABLE_ASYNC_UPDATES
        m_async_update_list_p = new async_update_list();
#   endif
}


// destructor

sc_prim_channel_registry::~sc_prim_channel_registry()
{
    delete m_async_update_list_p;
}

// called when construction is done

bool
sc_prim_channel_registry::construction_done()
{
    if( size() == m_construction_done )
        // nothing has been updated
        return true;

    for( ; m_construction_done < size(); ++m_construction_done ) {
        m_prim_channel_vec[m_construction_done]->construction_done();
    }

    return false;
}


// called when elaboration is done

void
sc_prim_channel_registry::elaboration_done()
{
    for( int i = 0; i < size(); ++ i ) {
	m_prim_channel_vec[i]->elaboration_done();
    }
}

// called before simulation begins

void
sc_prim_channel_registry::start_simulation()
{
    for( int i = 0; i < size(); ++ i ) {
	m_prim_channel_vec[i]->start_simulation();
    }
}

// called after simulation ends

void
sc_prim_channel_registry::simulation_done()
{
    for( int i = 0; i < size(); ++ i ) {
	m_prim_channel_vec[i]->simulation_done();
    }
}

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Andy Goodrich, Forte,
                               Bishnupriya Bhattacharya, Cadence Design Systems,
                               25 August, 2003

  Description of Modification: phase callbacks
    
 *****************************************************************************/


// $Log: sc_prim_channel.cpp,v $
// Revision 1.11  2011/08/26 21:38:32  acg
//  Philipp A. Hartmann: removed unused switch m_construction_done.
//
// Revision 1.10  2011/08/26 20:45:41  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.9  2011/08/24 22:05:36  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.8  2011/05/09 04:07:37  acg
//  Philipp A. Hartmann:
//    (1) Restore hierarchy in all phase callbacks.
//    (2) Ensure calls to before_end_of_elaboration.
//
// Revision 1.7  2011/04/19 02:36:26  acg
//  Philipp A. Hartmann: new aysnc_update and mutex support.
//
// Revision 1.6  2011/02/18 20:31:05  acg
//  Philipp A. Hartmann: added error messages for calls that cannot be done
//  after elaboration.
//
// Revision 1.5  2011/02/18 20:23:45  acg
//  Andy Goodrich: Copyright update.
//
// Revision 1.4  2011/02/14 17:50:16  acg
//  Andy Goodrich: testing for sc_port and sc_export instantiations during
//  end of elaboration and issuing appropriate error messages.
//
// Revision 1.3  2010/12/07 20:36:49  acg
//  Andy Goodrich: fix pointer that should have been initialized to zero.
//
// Revision 1.2  2010/12/07 19:50:36  acg
//  Andy Goodrich: addition of writer policies, courtesy of Philipp Hartmann.
//
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.4  2006/01/26 21:00:50  acg
//  Andy Goodrich: conversion to use sc_event::notify(SC_ZERO_TIME) instead of
//  sc_event::notify_delayed()
//
// Revision 1.3  2006/01/13 18:47:42  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

// Taf!
