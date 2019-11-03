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

  register_phase_callbacks.cpp -- Test for (un)registering dynamic callbacks

  Note: requires simulation phase callback support enabled in the kernel
        SC_ENABLE_SIMULATION_PHASE_CALLBACKS / --enable-phase-callbacks

  Original Author: Philipp A. Hartmann, OFFIS, 2013-05-17

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>

#define VERBOSE 1

SC_MODULE(phase_tracer)
{
  SC_HAS_PROCESS(phase_tracer);
  phase_tracer( sc_module_name nm
                  = sc_core::sc_gen_unique_name("phase_tracer") )
    : cb_count(0), timed_count(), delta_count()
  {
    SC_METHOD(timed);
    SC_METHOD(delta);
      sensitive << ev;

    old_mask = SC_STATUS_ANY;
    cb_mask = register_simulation_phase_callback( SC_STATUS_ANY );
    sc_assert( cb_mask == (old_mask & ~SC_ELABORATION & ~SC_RUNNING) );
    old_mask = cb_mask;

    cb_mask = unregister_simulation_phase_callback(SC_STOPPED);
    sc_assert( cb_mask == (old_mask & ~SC_STOPPED) );
    old_mask = cb_mask;

    cb_mask = register_simulation_phase_callback( SC_UNITIALIZED );
    sc_assert( cb_mask == old_mask );

    cb_mask = unregister_simulation_phase_callback(SC_UNITIALIZED);
    sc_assert( cb_mask == old_mask );

    cb_mask = unregister_simulation_phase_callback(SC_RUNNING);
    sc_assert( cb_mask == (old_mask & ~SC_END_OF_INITIALIZATION
//                                  & ~SC_END_OF_EVALUATION
                                    & ~SC_END_OF_UPDATE
                                    & ~SC_BEFORE_TIMESTEP) );
    old_mask = cb_mask;

    cb_mask = unregister_simulation_phase_callback(SC_ELABORATION);
    sc_assert( cb_mask == (old_mask & ~SC_BEFORE_END_OF_ELABORATION
                                    & ~SC_END_OF_ELABORATION ) );
    old_mask = cb_mask;

    cb_mask = unregister_simulation_phase_callback( SC_STATUS_ANY );
    sc_assert( cb_mask == SC_UNITIALIZED );
    old_mask = cb_mask;

    cb_mask = register_simulation_phase_callback( SC_RUNNING );
    sc_assert( cb_mask == ( SC_END_OF_INITIALIZATION
//                            | SC_END_OF_EVALUATION
                            | SC_END_OF_UPDATE | SC_BEFORE_TIMESTEP ) );

    cb_mask = register_simulation_phase_callback( SC_STATUS_ANY );
    sc_assert( cb_mask == (SC_STATUS_ANY & ~SC_ELABORATION & ~SC_RUNNING) );
  }

  void timed()
  {
    std::cout
      << sc_get_current_process_handle().name()
      << ": " << sc_time_stamp()
      << ": " << timed_count
      << std::endl;
    if( timed_count++ < 5 ) {
      next_trigger( 100, SC_NS );
    }
    if( delta_count < 5 )
      ev.notify( SC_ZERO_TIME );

    if( timed_count>=6 )
      sc_stop();
  }
  void delta()
  {
    std::cout
      << sc_get_current_process_handle().name()
      << ": " << sc_time_stamp()
      << ": " << delta_count
      << std::endl;
    delta_count++;
  }

  virtual void simulation_phase_callback()
  {
    cb_count++;

#   if VERBOSE
    {
      std::string ttp;
      if( !sc_pending_activity() ) {
        ttp = "MAX";
      } else {
        ttp = sc_time_to_pending_activity().to_string();
      }
      std::cout << name()
                << ": phase callback "
                << sc_get_status()
                << ": " << sc_time_stamp()
                << " -> pending activity: " << ttp
                << std::endl;
    }
#   endif
    sc_assert( cb_mask & sc_get_status() );

    switch( sc_get_status() )
    {
    case SC_END_OF_UPDATE:
    case SC_BEFORE_TIMESTEP:
      if( timed_count == 3 )
        ev.cancel();
      if( delta_count == 2 )
        ev.notify(SC_ZERO_TIME);
      if( timed_count == 2 )
        ev.notify( 1, SC_NS );
      break;
    default:
      // do nothing
      break;
    }
  }

  ~phase_tracer()
      { print_static_phase_stats( "[destructor]" ); }

  void print_static_phase_stats( const char* phase )
  {
#if VERBOSE
      std::cout << name()
                << ": " << phase << ": "
                << cb_count << " callbacks called."
                << std::endl;
#endif
  }

private:

  virtual void before_end_of_elaboration()
  {
    sc_assert( sc_get_status() == SC_BEFORE_END_OF_ELABORATION );
    print_static_phase_stats( "before_end_of_elaboration" );
  }

  virtual void end_of_elaboration()
  {
    sc_assert( sc_get_status() == SC_END_OF_ELABORATION );
    print_static_phase_stats( "end_of_elaboration" );
  }

  virtual void start_of_simulation()
  {
    sc_assert( sc_get_status() == SC_START_OF_SIMULATION );
    print_static_phase_stats( "start_of_simulation" );

    // ignored - issues warning
    register_simulation_phase_callback( SC_ELABORATION );
  }

  virtual void end_of_simulation()
  {
    sc_assert( sc_get_status() == SC_END_OF_SIMULATION );
    print_static_phase_stats( "end_of_simulation" );
  }



private:
  phase_cb_mask cb_mask, old_mask;
  sc_dt::uint64 cb_count, timed_count, delta_count;
  sc_event ev;
};


int sc_main(int, char*[])
{
  // don't run without callbacks enabled
  sc_report_handler::set_actions( "simulation phase callbacks not enabled"
                                , SC_DEFAULT_ERROR_ACTIONS );

  phase_tracer pt;
  sc_start();
  return 0;
}
