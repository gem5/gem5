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

// sc_pause.cpp -- test for 
//
//  Original Author: John Aynsley, Doulos, Inc.
//
// MODIFICATION LOG - modifiers, enter your name, affiliation, date and
//
// $Log: sc_pause.cpp,v $
// Revision 1.2  2011/05/08 19:18:46  acg
//  Andy Goodrich: remove extraneous + prefixes from git diff.
//

// sc_pause, sc_get_status, sc_is_running

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;
using std::hex;

struct my_interface: virtual sc_interface
{
  virtual void schedule_events_while_paused() = 0;
};

struct M: sc_module, private my_interface
{
  sc_export<my_interface> xport;
  
  SC_CTOR(M)
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " CTOR in " << name() << endl;
    sc_assert( sc_get_status() == SC_BEFORE_END_OF_ELABORATION );
    sc_assert( sc_is_running() == false );
    
    xport.bind(*this);
    
    sc_spawn(sc_bind(&M::T, this));
    
    SC_THREAD(on_immed_event);
    SC_THREAD(on_delta_event);
    SC_THREAD(on_timed_event);
  }
  
  sc_signal<int> sig1, sig2, sig3, sig4;
  sc_event immed_event;
  sc_event delta_event;
  sc_event timed_event;

  void before_end_of_elaboration()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " before_end_of_elaboration in " << name() << endl;
    sc_assert( sc_get_status() == SC_BEFORE_END_OF_ELABORATION );
    sc_assert( sc_is_running() == false );
    sig1.write(1);
    timed_event.notify(1234, SC_NS);
    sc_pause();  // Should be ignored
  }
  
  void end_of_elaboration()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " end_of_elaboration in " << name() << endl;
    sc_assert( sc_get_status() == SC_END_OF_ELABORATION );
    sc_assert( sc_is_running() == false );
    sig2.write(2);
    sc_pause();  // Should be ignored
  }
  
  void start_of_simulation()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " start_of_simulation in " << name() << endl;
    sc_assert( sc_get_status() == SC_START_OF_SIMULATION );
    sc_assert( sc_is_running() == false );
    sig3.write(3);
    sig4.write(0);
    sc_pause();  // Should be ignored
  }
  
  void end_of_simulation()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " end_of_simulation in " << name() << endl;;
    sc_assert( sc_get_status() == SC_END_OF_SIMULATION );
    sc_assert( sc_is_running() == false );
  }
  
  void T()
  {
    sc_assert( sig1.read() == 1 );
    sc_assert( sig2.read() == 2 );
    sc_assert( sig3.read() == 3 );
  }
  
  void on_immed_event()
  {
    wait(immed_event);
    cout << "on_immed_event() awoke\n";

    // Should run in 1st eval phase after pause
    sc_assert( sig4.read() == 0 );
  }
  
  void on_delta_event()
  {
    wait(delta_event);
    cout << "on_delta_event() awoke\n";
    
    // Should run in 2nd eval phase after pause
    sc_assert( sig4.read() == 4 );
  }
  
  void on_timed_event()
  {
    wait(timed_event);
    cout << "on_timed_event() awoke\n";
    
    sc_assert( sig1.read() == 1 );
    sc_assert( sig2.read() == 2 );
    sc_assert( sig3.read() == 3 );
    sc_assert( sig4.read() == 0 );
    sc_assert( sc_time_stamp() == sc_time(1234, SC_NS) );
  }

private:
  
  void schedule_events_while_paused()
  {
    sig4.write(4);
    immed_event.notify();
    delta_event.notify(SC_ZERO_TIME);
    
    // Should be able to instantiate an sc_object while paused
    mut = new sc_mutex("mut");
    sem = new sc_semaphore("sem", 1);
  }
  
  sc_mutex* mut;
  sc_semaphore* sem;
};

SC_MODULE(Top)
{
  SC_CTOR(Top)
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " CTOR in " << name() << endl;
    sc_assert( sc_get_status() == SC_ELABORATION );
    sc_assert( sc_is_running() == false );
    SC_THREAD(T);
    
    sc_spawn_options opt;
      opt.spawn_method();
      opt.set_sensitivity( &timed_ev );
      opt.set_sensitivity( &delta_ev );
      opt.dont_initialize();
    sc_spawn(sc_bind(&Top::ev_handler, this), "ev_handler", &opt);
    
    SC_METHOD(immed_ev_handler);
      sensitive << immed_ev;
      dont_initialize();

    immed_ev_delta = 0;
    sc_assert( sc_delta_count() == 0 );
  }
  
  ~Top()
  {
    sc_assert( sc_get_status() == SC_STOPPED );
    sc_assert( sc_is_running() == false );
  }

  M* m;
  sc_event timed_ev;
  sc_event delta_ev;
  sc_event immed_ev;
  unsigned immed_ev_delta;
  sc_signal<int> sig;
    
  void before_end_of_elaboration()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " before_end_of_elaboration in " << name() << endl;
    sc_assert( sc_get_status() == SC_BEFORE_END_OF_ELABORATION );
    sc_assert( sc_is_running() == false );

    m = new M("m");
  }
  
  void end_of_elaboration()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " end_of_elaboration in " << name() << endl;
    sc_assert( sc_get_status() == SC_END_OF_ELABORATION );
    sc_assert( sc_is_running() == false );
  }
  
  void start_of_simulation()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " start_of_simulation in " << name() << endl;
    sc_assert( sc_get_status() == SC_START_OF_SIMULATION );
    sc_assert( sc_is_running() == false );
  }
  
  void end_of_simulation()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " end_of_simulation in " << name() << endl;
    sc_assert( sc_get_status() == SC_END_OF_SIMULATION );
    sc_assert( sc_is_running() == false );
    sc_assert( immed_ev_delta == 999 );
  }
  
  void T()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " PROCESS in " << name() << endl;
    sc_assert( sc_delta_count() == 0 );
    sc_assert( sig.read()      == 42 );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    wait(timed_ev);
    sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    
    sc_pause();
    cout << "sc_get_status() == " << hex << sc_get_status() << " PROCESS after sc_pause in " << name() << endl;
    sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    wait(SC_ZERO_TIME);
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    
    sc_pause();
    cout << "sc_get_status() == " << hex << sc_get_status() << " PROCESS after sc_pause in " << name() << endl;
    sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    wait(2, SC_US);
    sc_assert( sc_time_stamp() == sc_time(44, SC_US) );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    
    sc_stop();
    cout << "sc_get_status() == " << hex << sc_get_status() << " PROCESS after sc_stop() in " << name() << endl;;
    sc_assert( sc_time_stamp() == sc_time(44, SC_US) );
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sc_is_running() == true );
    
    sc_pause();
    sc_assert( sc_get_status() == SC_RUNNING );
  }
  
  void ev_handler()
  {
    cout << "sc_get_status() == " << hex << sc_get_status() << " METHOD in " << name() << endl;
    sc_assert( sc_get_status() == SC_RUNNING );
    sc_assert( sig.read()      == 42 );

    static bool first = true;
    if (first)
    {
      sc_assert( sc_time_stamp() == SC_ZERO_TIME );
      first = false;
    }
    else
      sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
  }
  
  void immed_ev_handler()
  {
    sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
    sc_assert( sc_delta_count() == immed_ev_delta );
    immed_ev_delta = 999;
  }
};

void spawned_while_paused()
{
  cout << "spawned_while_paused() awoke" << endl;
  sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
  sc_assert( sc_get_status() == SC_RUNNING );
  sc_assert( sc_pending_activity() == true );
}

int sc_main(int argc, char* argv[])
{
  sc_assert( sc_delta_count() == 0 );
  cout << "sc_get_status() == " << hex << sc_get_status() << " ELAB" << endl;
  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_is_running() == false );
  Top top("top");
  
  sc_pause();  // Should be ignored
  
  // Schedule some update requests and events
  top.sig.write(42);
  top.timed_ev.notify(42, SC_US);
  top.delta_ev.notify(SC_ZERO_TIME);
  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_delta_count() == 0 );

  sc_start();
  cout << "sc_get_status() == " << hex << sc_get_status() << " PAUSED" << endl;
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_is_running() == true );
  sc_assert( sc_time_stamp() == sc_time(42, SC_US) );
  
  sc_start(1, SC_US);
  cout << "sc_get_status() == " << hex << sc_get_status() << " PAUSED" << endl;
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_is_running() == true );
  sc_assert( sc_time_stamp() == sc_time(42, SC_US) );

  // Schedule an immediate notification
  top.immed_ev.notify();
  top.immed_ev_delta = sc_delta_count();

  // IMC while paused  
  top.m->xport->schedule_events_while_paused();
  
  sc_process_handle h = sc_spawn(&spawned_while_paused);
  sc_assert( h.valid() );

  sc_start();
  cout << "sc_get_status() == " << hex << sc_get_status() << " STOPPED" << endl;
  sc_assert( sc_get_status() == SC_STOPPED );
  sc_assert( sc_is_running() == false );
  sc_assert( sc_time_stamp() == sc_time(44, SC_US) );

  cout << endl << "Success" << endl;
  return 0;
}
