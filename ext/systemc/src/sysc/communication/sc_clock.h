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

  sc_clock.h -- The clock channel.

  Original Author: Martin Janssen, Synopsys, Inc., 2001-05-21

  CHANGE LOG IS AT THE END OF THE FILE
 *****************************************************************************/

#ifndef SC_CLOCK_H
#define SC_CLOCK_H


#include "sysc/kernel/sc_module.h"
#include "sysc/communication/sc_signal.h"
#include "sysc/tracing/sc_trace.h"

namespace sc_core {

// ----------------------------------------------------------------------------
//  CLASS : sc_clock
//
//  The clock channel.
// ----------------------------------------------------------------------------

class sc_clock
  : public sc_signal<bool,SC_ONE_WRITER>
{
  typedef sc_signal<bool,SC_ONE_WRITER> base_type;
public:

    friend class sc_clock_posedge_callback;
    friend class sc_clock_negedge_callback;

    // constructors

    sc_clock();

    explicit sc_clock( const char* name_ );

    sc_clock( const char* name_,
	      const sc_time& period_,
	      double         duty_cycle_ = 0.5,
	      const sc_time& start_time_ = SC_ZERO_TIME,
	      bool           posedge_first_ = true );

    sc_clock( const char* name_,
	      double         period_v_,
	      sc_time_unit   period_tu_,
	      double         duty_cycle_ = 0.5 );

    sc_clock( const char* name_,
	      double         period_v_,
	      sc_time_unit   period_tu_,
	      double         duty_cycle_,
	      double         start_time_v_,
	      sc_time_unit   start_time_tu_,
	      bool           posedge_first_ = true );

    // for backward compatibility with 1.0
    sc_clock( const char* name_,
	      double         period_,            // in default time units
	      double         duty_cycle_ = 0.5,
	      double         start_time_ = 0.0,  // in default time units
	      bool           posedge_first_ = true );

    // destructor (does nothing)
    virtual ~sc_clock();

    virtual void register_port( sc_port_base&, const char* if_type );
    virtual void write( const bool& );

    // get the period
    const sc_time& period() const
	{ return m_period; }

    // get the duty cycle
    double duty_cycle() const
	{ return m_duty_cycle; }


    // get the current time / clock characteristics

    bool posedge_first() const
        { return m_posedge_first; }

    sc_time start_time() const
        { return m_start_time; }

    static const sc_time& time_stamp();

    virtual const char* kind() const
        { return "sc_clock"; }


#if 0 // @@@@#### REMOVE
    // for backward compatibility with 1.0

    sc_signal_in_if<bool>& signal()
	{ return *this; }

    const sc_signal_in_if<bool>& signal() const
	{ return *this; }

    static void start( const sc_time& duration )
	{ sc_start( duration ); }

    static void start( double v, sc_time_unit tu )
	{ sc_start( sc_time(v, tu) ); }

    static void start( double duration = -1 )
	{ sc_start( duration ); }

    static void stop()
	{ sc_stop(); }
#endif

protected:

    void before_end_of_elaboration();

    // processes
    void posedge_action();
    void negedge_action();


    // error reporting
    void report_error( const char* id, const char* add_msg = 0 ) const;


    void init( const sc_time&, double, const sc_time&, bool );

    bool is_clock() const { return true; }

protected:

    sc_time  m_period;		// the period of this clock
    double   m_duty_cycle;	// the duty cycle (fraction of period)
    sc_time  m_start_time;	// the start time of the first edge
    bool     m_posedge_first;   // true if first edge is positive
    sc_time  m_posedge_time;	// time till next positive edge
    sc_time  m_negedge_time;	// time till next negative edge

    sc_event m_next_posedge_event;
    sc_event m_next_negedge_event;

private:

    // disabled
    sc_clock( const sc_clock& );
    sc_clock& operator = ( const sc_clock& );
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

// processes

inline
void
sc_clock::posedge_action()
{
    m_next_negedge_event.notify_internal( m_negedge_time );
	m_new_val = true;
	request_update();
}

inline
void
sc_clock::negedge_action()
{
    m_next_posedge_event.notify_internal( m_posedge_time );
	m_new_val = false;
	request_update();
}


// ----------------------------------------------------------------------------

class sc_clock_posedge_callback {
public:
    sc_clock_posedge_callback(sc_clock* target_p) : m_target_p(target_p) {}
    inline void operator () () { m_target_p->posedge_action(); }
  protected:
    sc_clock* m_target_p;
};

class sc_clock_negedge_callback {
  public:
    sc_clock_negedge_callback(sc_clock* target_p) : m_target_p(target_p) {}
    inline void operator () () { m_target_p->negedge_action(); }
  protected:
    sc_clock* m_target_p;
};


} // namespace sc_core

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Bishnupriya Bhattacharya, Cadence Design Systems,
                               3 October, 2003
  Description of Modification: sc_clock inherits from sc_signal<bool> only
                               instead of sc_signal_in_if<bool> and sc_module.
    
      Name, Affiliation, Date:
  Description of Modification:
    
 *****************************************************************************/
//$Log: sc_clock.h,v $
//Revision 1.5  2011/08/26 20:45:39  acg
// Andy Goodrich: moved the modification log to the end of the file to
// eliminate source line number skew when check-ins are done.
//
//Revision 1.4  2011/08/24 22:05:35  acg
// Torsten Maehne: initialization changes to remove warnings.
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
//Revision 1.5  2006/01/25 00:31:11  acg
// Andy Goodrich: Changed over to use a standard message id of
// SC_ID_IEEE_1666_DEPRECATION for all deprecation messages.
//
//Revision 1.4  2006/01/24 20:43:25  acg
// Andy Goodrich: convert notify_delayed() calls into notify_internal() calls.
// notify_internal() is an implementation dependent version of notify_delayed()
// that is simpler, and does not trigger the deprecation warning one would get
// using notify_delayed().
//
//Revision 1.3  2006/01/18 21:42:26  acg
//Andy Goodrich: Changes for check writer support, and tightening up sc_clock
//port usage.
//
//Revision 1.2  2006/01/03 23:18:26  acg
//Changed copyright to include 2006.
//
//Revision 1.1.1.1  2005/12/19 23:16:43  acg
//First check in of SystemC 2.1 into its own archive.
//
//Revision 1.14  2005/06/10 22:43:55  acg
//Added CVS change log annotation.
//

#endif

// Taf!
