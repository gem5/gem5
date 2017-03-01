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

  sc_prim_channel.h -- Abstract base class of all primitive channel classes.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

 CHANGE LOG AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_PRIM_CHANNEL_H
#define SC_PRIM_CHANNEL_H

#include "sysc/kernel/sc_object.h"
#include "sysc/kernel/sc_wait.h"
#include "sysc/kernel/sc_wait_cthread.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel
//
//  Abstract base class of all primitive channel classes.
// ----------------------------------------------------------------------------

class sc_prim_channel
: public sc_object
{
    friend class sc_prim_channel_registry;

public:
    enum { list_end = 0xdb };
public:
    virtual const char* kind() const
        { return "sc_prim_channel"; }

    inline bool update_requested() 
	{ return m_update_next_p != (sc_prim_channel*)list_end; }

    // request the update method to be executed during the update phase
    inline void request_update();

    // request the update method to be executed during the update phase
    // from a process external to the simulator.
    void async_request_update();

protected:

    // constructors
    sc_prim_channel();
    explicit sc_prim_channel( const char* );

    // destructor
    virtual ~sc_prim_channel();

    // the update method (does nothing by default)
    virtual void update();

    // called by construction_done (does nothing by default)
    virtual void before_end_of_elaboration();

    // called by elaboration_done (does nothing by default)
    virtual void end_of_elaboration();

    // called by start_simulation (does nothing by default)
    virtual void start_of_simulation();

    // called by simulation_done (does nothing by default)
    virtual void end_of_simulation();

protected:

    // to avoid calling sc_get_curr_simcontext()

    // static sensitivity for SC_THREADs and SC_CTHREADs

    void wait()
        { sc_core::wait( simcontext() ); }


    // dynamic sensitivity for SC_THREADs and SC_CTHREADs

    void wait( const sc_event& e )
        { sc_core::wait( e, simcontext() ); }

    void wait( const sc_event_or_list& el )
	{ sc_core::wait( el, simcontext() ); }

    void wait( const sc_event_and_list& el )
	{ sc_core::wait( el, simcontext() ); }

    void wait( const sc_time& t )
        { sc_core::wait( t, simcontext() ); }

    void wait( double v, sc_time_unit tu )
        { sc_core::wait( sc_time( v, tu, simcontext() ), simcontext() ); }

    void wait( const sc_time& t, const sc_event& e )
        { sc_core::wait( t, e, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event& e )
        { sc_core::wait( sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void wait( const sc_time& t, const sc_event_or_list& el )
        { sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_or_list& el )
        { sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void wait( const sc_time& t, const sc_event_and_list& el )
        { sc_core::wait( t, el, simcontext() ); }

    void wait( double v, sc_time_unit tu, const sc_event_and_list& el )
        { sc_core::wait( sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void wait( int n )
        { sc_core::wait( n, simcontext() ); }


    // static sensitivity for SC_METHODs

    void next_trigger()
	{ sc_core::next_trigger( simcontext() ); }


    // dynamic sensitivity for SC_METHODs

    void next_trigger( const sc_event& e )
        { sc_core::next_trigger( e, simcontext() ); }

    void next_trigger( const sc_event_or_list& el )
        { sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_event_and_list& el )
        { sc_core::next_trigger( el, simcontext() ); }

    void next_trigger( const sc_time& t )
        { sc_core::next_trigger( t, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu )
        {sc_core::next_trigger( sc_time( v, tu, simcontext() ), simcontext() );}

    void next_trigger( const sc_time& t, const sc_event& e )
        { sc_core::next_trigger( t, e, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event& e )
        { sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), e, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_or_list& el )
        { sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_or_list& el )
        { sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }

    void next_trigger( const sc_time& t, const sc_event_and_list& el )
        { sc_core::next_trigger( t, el, simcontext() ); }

    void next_trigger( double v, sc_time_unit tu, const sc_event_and_list& el )
        { sc_core::next_trigger( 
	    sc_time( v, tu, simcontext() ), el, simcontext() ); }


    // for SC_METHODs and SC_THREADs and SC_CTHREADs

    bool timed_out()
	{ return sc_core::timed_out( simcontext() ); }


#if 0 // @@@@####
    // delta count maintenance
    sc_dt::uint64 delta_count()
	{ return simcontext()->m_delta_count; }
#endif

private:

    // called during the update phase of a delta cycle (if requested)
    void perform_update();

    // called when construction is done
    void construction_done();

    // called when elaboration is done
    void elaboration_done();

    // called before simulation starts
    void start_simulation();

    // called after simulation ends
    void simulation_done();

    // disabled
    sc_prim_channel( const sc_prim_channel& );
    sc_prim_channel& operator = ( const sc_prim_channel& );

private:

    sc_prim_channel_registry* m_registry;          // Update list manager.
    sc_prim_channel*          m_update_next_p;     // Next entry in update list.
};


// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel_registry
//
//  Registry for all primitive channels.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

class sc_prim_channel_registry
{
    friend class sc_simcontext;

public:

    void insert( sc_prim_channel& );
    void remove( sc_prim_channel& );


    int size() const
        { return m_prim_channel_vec.size(); }

    inline void request_update( sc_prim_channel& );
    void async_request_update( sc_prim_channel& );

    bool pending_updates() const
    { 
        return m_update_list_p != (sc_prim_channel*)sc_prim_channel::list_end 
               || pending_async_updates();
    }   

    bool pending_async_updates() const;

private:

    // constructor
    explicit sc_prim_channel_registry( sc_simcontext& simc_ );

    // destructor
    ~sc_prim_channel_registry();

    // called during the update phase of a delta cycle
    void perform_update();

    // called when construction is done
    bool construction_done();

    // called when elaboration is done
    void elaboration_done();

    // called before simulation starts
    void start_simulation();

    // called after simulation ends
    void simulation_done();

    // disabled
    sc_prim_channel_registry();
    sc_prim_channel_registry( const sc_prim_channel_registry& );
    sc_prim_channel_registry& operator = ( const sc_prim_channel_registry& );

private:
    class async_update_list;   

    async_update_list*            m_async_update_list_p; // external updates.
    int                           m_construction_done;   // # of constructs.
    std::vector<sc_prim_channel*> m_prim_channel_vec;    // existing channels.
    sc_simcontext*                m_simc;                // simulator context.
    sc_prim_channel*              m_update_list_p;       // internal updates.
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel_registry
//
//  Registry for all primitive channels.
//  FOR INTERNAL USE ONLY!
// ----------------------------------------------------------------------------

inline
void
sc_prim_channel_registry::request_update( sc_prim_channel& prim_channel_ )
{
    prim_channel_.m_update_next_p = m_update_list_p;
    m_update_list_p = &prim_channel_;
}

// ----------------------------------------------------------------------------
//  CLASS : sc_prim_channel
//
//  Abstract base class of all primitive channel classes.
// ----------------------------------------------------------------------------

// request the update method (to be executed during the update phase)

inline
void
sc_prim_channel::request_update()
{
    if( ! m_update_next_p ) {
	m_registry->request_update( *this );
    }
}

// request the update method from external to the simulator (to be executed 
// during the update phase)

inline
void
sc_prim_channel::async_request_update()
{
    m_registry->async_request_update(*this);
}


// called during the update phase of a delta cycle (if requested)

inline
void
sc_prim_channel::perform_update()
{
    update();
    m_update_next_p = 0;
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
//$Log: sc_prim_channel.h,v $
//Revision 1.10  2011/08/26 21:38:32  acg
// Philipp A. Hartmann: removed unused switch m_construction_done.
//
//Revision 1.9  2011/08/07 19:08:01  acg
// Andy Goodrich: moved logs to end of file so line number synching works
// better between versions.
//
//Revision 1.8  2011/05/09 04:07:37  acg
// Philipp A. Hartmann:
//   (1) Restore hierarchy in all phase callbacks.
//   (2) Ensure calls to before_end_of_elaboration.
//
//Revision 1.7  2011/05/05 17:44:01  acg
// Philip A. Hartmann: change in the name of pending_async_updates.
//
//Revision 1.6  2011/04/19 15:03:48  acg
// Philipp A. Hartmann: remove ASYNC_UPDATE preprocessor check from header.
//
//Revision 1.5  2011/04/19 02:36:26  acg
// Philipp A. Hartmann: new aysnc_update and mutex support.
//
//Revision 1.4  2011/04/05 20:48:09  acg
// Andy Goodrich: changes to make sure that event(), posedge() and negedge()
// only return true if the clock has not moved.
//
//Revision 1.3  2011/02/18 20:23:45  acg
// Andy Goodrich: Copyright update.
//
//Revision 1.2  2011/01/20 16:52:15  acg
// Andy Goodrich: changes for IEEE 1666 2011.
//
//Revision 1.1.1.1  2006/12/15 20:20:04  acg
//SystemC 2.3
//
//Revision 1.3  2006/05/08 17:52:47  acg
// Andy Goodrich:
//   (1) added David Long's forward declarations for friend functions,
//       methods, and operators to keep the Microsoft compiler happy.
//   (2) Added delta_count() method to sc_prim_channel for use by
//       sc_signal so that the friend declaration in sc_simcontext.h
//	   can be for a non-templated class (i.e., sc_prim_channel.)
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.10  2005/07/30 03:44:11  acg
//Changes from 2.1.
//
//Revision 1.9  2005/06/10 22:43:55  acg
//Added CVS change log annotation.

#endif

// Taf!
