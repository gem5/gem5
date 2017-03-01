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

  sc_clock.cpp -- The clock channel.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

// using notify_delayed().
//
// Revision 1.4  2006/01/18 21:42:26  acg
// Andy Goodrich: Changes for check writer support, and tightening up sc_clock
// port usage.
//
// Revision 1.3  2006/01/13 18:47:41  acg
// Added $Log command so that CVS comments are reproduced in the source.
//

#include "sysc/communication/sc_clock.h"
#include "sysc/communication/sc_communication_ids.h"
#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_process.h"
#include "sysc/kernel/sc_spawn.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_clock
//
//  The clock channel.
// ----------------------------------------------------------------------------

// constructors

sc_clock::sc_clock() : 
    base_type( sc_gen_unique_name( "clock" ) ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
                          "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
                          "_next_negedge_event").c_str())

{
    init( sc_time::from_value(simcontext()->m_time_params->default_time_unit),
	  0.5,
	  SC_ZERO_TIME,
	  true );

    m_next_posedge_event.notify_internal( m_start_time );
}

sc_clock::sc_clock( const char* name_ ) :
    base_type( name_ ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_negedge_event").c_str())
{
    init( sc_time::from_value(simcontext()->m_time_params->default_time_unit),
	  0.5,
	  SC_ZERO_TIME,
	  true );

    m_next_posedge_event.notify_internal( m_start_time );
}

sc_clock::sc_clock( const char* name_,
		    const sc_time& period_,
		    double         duty_cycle_,
		    const sc_time& start_time_,
		    bool           posedge_first_ ) :
    base_type( name_ ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_negedge_event").c_str())
{
    init( period_,
	  duty_cycle_,
	  start_time_,
	  posedge_first_ );

    if( posedge_first_ ) {
	// posedge first
	m_next_posedge_event.notify_internal( m_start_time );
    } else {
	// negedge first
	m_next_negedge_event.notify_internal( m_start_time );
    }
}

sc_clock::sc_clock( const char* name_,
		    double         period_v_,
		    sc_time_unit   period_tu_,
		    double         duty_cycle_ ) :
    base_type( name_ ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_negedge_event").c_str())
{
    init( sc_time( period_v_, period_tu_, simcontext() ),
	  duty_cycle_,
	  SC_ZERO_TIME,
	  true );

    // posedge first
    m_next_posedge_event.notify_internal( m_start_time );
}

sc_clock::sc_clock( const char* name_,
		    double         period_v_,
		    sc_time_unit   period_tu_,
		    double         duty_cycle_,
		    double         start_time_v_,
		    sc_time_unit   start_time_tu_,
		    bool           posedge_first_ ) :
    base_type( name_ ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_negedge_event").c_str())
{
    init( sc_time( period_v_, period_tu_, simcontext() ),
	  duty_cycle_,
	  sc_time( start_time_v_, start_time_tu_, simcontext() ),
	  posedge_first_ );

    if( posedge_first_ ) {
	// posedge first
	m_next_posedge_event.notify_internal( m_start_time );
    } else {
	// negedge first
	m_next_negedge_event.notify_internal( m_start_time );
    }
}

// for backward compatibility with 1.0
sc_clock::sc_clock( const char* name_,
		    double         period_,      // in default time units
		    double         duty_cycle_,
		    double         start_time_,  // in default time units
		    bool           posedge_first_ ) :
    base_type( name_ ),
    m_period(), m_duty_cycle(), m_start_time(), m_posedge_first(),
    m_posedge_time(), m_negedge_time(),
    m_next_posedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_posedge_event").c_str()),
    m_next_negedge_event( (std::string(SC_KERNEL_EVENT_PREFIX) + 
			   std::string(name_) + "_next_negedge_event").c_str())
{
    static bool warn_sc_clock=true;
    if ( warn_sc_clock )
    {
        warn_sc_clock = false;
	SC_REPORT_INFO(SC_ID_IEEE_1666_DEPRECATION_, 
	   "\n    sc_clock(const char*, double, double, double, bool)\n"
	   "    is deprecated use a form that includes sc_time or\n"
	   "    sc_time_unit");
    }

    sc_time default_time =
      sc_time::from_value( simcontext()->m_time_params->default_time_unit );

    init( ( period_ * default_time ),
	  duty_cycle_,
	  ( start_time_ * default_time ),
	  posedge_first_ );

    if( posedge_first_ ) {
	// posedge first
	m_next_posedge_event.notify_internal( m_start_time );
    } else {
	// negedge first
	m_next_negedge_event.notify_internal( m_start_time );
    }
}


//------------------------------------------------------------------------------
//"sc_clock::before_end_of_elaboration"
//
// This callback is used to spawn the edge processes for this object instance.
// The processes are created here rather than the constructor for the object
// so that the processes are registered with the global simcontext rather
// than the scope of the clock's parent.
//------------------------------------------------------------------------------
#if ( defined(_MSC_VER) && _MSC_VER < 1300 ) //VC++6.0 doesn't support sc_spawn with functor.
#   define sc_clock_posedge_callback(ptr) sc_clock_posedge_callback

#   define sc_clock_negedge_callback(ptr) sc_clock_negedge_callback

#   define sc_spawn(a,b,c) { \
        sc_process_handle result(new sc_spawn_object<a>(a(this),b,c)); \
    }
#endif // ( defined(_MSC_VER) && _MSC_VER < 1300 )

void sc_clock::before_end_of_elaboration()
{
    std::string gen_base;
    sc_spawn_options posedge_options;	// Options for posedge process.
    sc_spawn_options negedge_options;	// Options for negedge process.

    posedge_options.spawn_method();
    posedge_options.dont_initialize();
    posedge_options.set_sensitivity(&m_next_posedge_event);
    gen_base = basename();
    gen_base += "_posedge_action";
    sc_spawn(sc_clock_posedge_callback(this),
	sc_gen_unique_name( gen_base.c_str() ), &posedge_options);

    negedge_options.spawn_method();
    negedge_options.dont_initialize();
    negedge_options.set_sensitivity(&m_next_negedge_event);
    gen_base = basename();
    gen_base += "_negedge_action";
    sc_spawn( sc_clock_negedge_callback(this),
    	sc_gen_unique_name( gen_base.c_str() ), &negedge_options );
}

//clear VC++6.0 macros
#undef sc_clock_posedge_callback
#undef sc_clock_negedge_callback
#undef sc_spawn

// destructor (does nothing)

sc_clock::~sc_clock()
{}

void sc_clock::register_port( sc_port_base& /*port*/, const char* if_typename_ )
{
    std::string nm( if_typename_ );
    if( nm == typeid( sc_signal_inout_if<bool> ).name() ) {
	    SC_REPORT_ERROR(SC_ID_ATTEMPT_TO_BIND_CLOCK_TO_OUTPUT_, "");
    }
}

void
sc_clock::write( const bool& /* value */ )
{
    SC_REPORT_ERROR(SC_ID_ATTEMPT_TO_WRITE_TO_CLOCK_, "");
}

// interface methods

// get the current time

const sc_time&
sc_clock::time_stamp()
{
    return sc_time_stamp();
}


// error reporting

void
sc_clock::report_error( const char* id, const char* add_msg ) const
{
    char msg[BUFSIZ];
    if( add_msg != 0 ) {
	std::sprintf( msg, "%s: clock '%s'", add_msg, name() );
    } else {
	std::sprintf( msg, "clock '%s'", name() );
    }
    SC_REPORT_ERROR( id, msg );
}


void
sc_clock::init( const sc_time& period_,
		double         duty_cycle_,
		const sc_time& start_time_,
		bool           posedge_first_ )
{
    if( period_ == SC_ZERO_TIME ) {
	report_error( SC_ID_CLOCK_PERIOD_ZERO_,
		      "increase the period" );
    }
    m_period = period_;
    m_posedge_first = posedge_first_;

    if( duty_cycle_ <= 0.0 || duty_cycle_ >= 1.0 ) {
	m_duty_cycle = 0.5;
    } else {
	m_duty_cycle = duty_cycle_;
    }

    m_negedge_time = m_period * m_duty_cycle;
    m_posedge_time = m_period - m_negedge_time;

    if( m_negedge_time == SC_ZERO_TIME ) {
	report_error( SC_ID_CLOCK_HIGH_TIME_ZERO_,
		      "increase the period or increase the duty cycle" );
    }
    if( m_posedge_time == SC_ZERO_TIME ) {
	report_error( SC_ID_CLOCK_LOW_TIME_ZERO_,
		      "increase the period or decrease the duty cycle" );
    }

    if( posedge_first_ ) {
	this->m_cur_val = false;
	this->m_new_val = false;
    } else {
	this->m_cur_val = true;
	this->m_new_val = true;
    }

    m_start_time = start_time_;

}

} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               Andy Goodrich, Forte Design Systems,
                               3 October, 2003
  Description of Modification: sc_clock inherits from sc_signal<bool> only
                               instead of sc_signal_in_if<bool> and sc_module.
                               The 2 methods posedge_action() and
                               negedge_action() are created using sc_spawn().
                               boost::bind() is not required, instead a local
                               bind function can be used since the signatures
                               of the spawned functions are statically known.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// $Log: sc_clock.cpp,v $
// Revision 1.7  2011/08/26 20:45:39  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.6  2011/08/24 22:05:35  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.5  2011/08/15 16:43:24  acg
//  Torsten Maehne: changes to remove unused argument warnings.
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
// Revision 1.8  2006/04/18 23:36:50  acg
//  Andy Goodrich: made add_trace_internal public until I can figure out
//  how to do a friend specification for sc_trace in an environment where
//  there are partial template and full template specifications for its
//  arguments.
//
// Revision 1.7  2006/04/17 16:38:42  acg
//  Andy Goodrich: added more context to the deprecation message for the
//  sc_clock constructor.
//
// Revision 1.6  2006/01/25 00:31:11  acg
//  Andy Goodrich: Changed over to use a standard message id of
//  SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
// Revision 1.5  2006/01/24 20:43:24  acg
// Andy Goodrich: convert notify_delayed() calls into notify_internal() calls.
// notify_internal() is an implementation dependent version of notify_delayed()
// that is simpler, and does not trigger the deprecation warning one would get

// Taf!
