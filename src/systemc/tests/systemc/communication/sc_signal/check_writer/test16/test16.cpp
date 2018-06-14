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

  test16.cpp -- sc_writer_policy: test SC_SIGNAL_WRITE_CHECK=CONFLICT

  Original Author: Philipp A. Hartmann, Intel, 2017-05-12

 *****************************************************************************/

// see https://github.com/OSCI-WG/systemc/issues/222

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>
#include <iomanip>
#include <sstream>

using sc_core::SC_NS;
using sc_core::SC_ZERO_TIME;

SC_MODULE(dut)
{
    static const int num_drivers = 3;

    SC_CTOR(dut)
      : sig("sig")
    {
        SC_THREAD(run);
        for(int id=1; id <= num_drivers; ++id) {
          // first and last driver write the same values
          spawn_driver( id, (id-1) % (num_drivers-1) + 1 );
        }
    }

private:
    void spawn_driver(int id, int value) {
      sc_assert( id > 0 && id <= num_drivers );
      sc_core::sc_spawn_options sp;
      sp.set_sensitivity(&ev[id-1]);
      sp.spawn_method();
      sp.dont_initialize();

      std::stringstream nm;
      nm << "p" << id;
      sc_spawn( sc_bind(&dut::driver, this, value), nm.str().c_str(), &sp );
    }

    void run() {
      wait(1, SC_NS);

      std::cout << "\n*** trigger each driver in individual delta cycles" << std::endl;
      for(int id=1; id <= num_drivers; ++id) {
        trigger(id); // NO error expected (environment override)
        wait(SC_ZERO_TIME);
        log();
      }

      wait(1, SC_NS);
      std::cout << "\n*** trigger 1-2-3 in the same delta cycle" << std::endl;
      trigger(1);
      trigger(2); // error expected
      trigger(3); // error expected
      wait(SC_ZERO_TIME);
      log();

      wait(1, SC_NS);
      std::cout << "\n*** trigger 2-3-1 in the same delta cycle" << std::endl;
      trigger(2);
      trigger(3); // error expected
      trigger(1); // error expected
      wait(SC_ZERO_TIME);
      log();

      wait(1, SC_NS);
      std::cout << "\n*** trigger 3-1-2 in the same delta cycle" << std::endl;
      trigger(3);
      trigger(1); // error expected
      trigger(2); // error expected
      wait(SC_ZERO_TIME);
      log();

      wait(1, SC_NS);
      std::cout << "\n*** trigger 1-1-2-2 in the same delta cycle" << std::endl;
      trigger(1);
      trigger(1); // NO error expected (current process)
      trigger(2); // error expected
      trigger(2); // NO error expected (current process)
      wait(SC_ZERO_TIME);
      log();
    }

    void trigger(int id) {
      sc_assert( id > 0 && id <= num_drivers );
      ev[id-1].notify();
      wait(ev_schedule);
    }

    void driver(int value) {
        log(value);
        sig.write(value); // errors suppressed via report handler below
        ev_schedule.notify();
    }

    void log(int value = -1) {
        std::cout
          << "\n"
          << std::setw(8) << sc_core::sc_get_current_process_handle().name() << ": "
          << std::setw(5) << sc_core::sc_time_stamp()
          << " @ " << std::setw(2) << sc_core::sc_delta_count() << ": "
          << ( (value!=-1) ? "writing " : "reading " )
          << sig.name() << " = "
          << ( (value!=-1) ? value : sig.read() )
          << std::endl;
    }

    sc_core::sc_signal<int, sc_core::SC_ONE_WRITER> sig; // use single-writer signal
    sc_core::sc_event ev_schedule, ev[num_drivers];
};

#ifdef _WIN32
#define putenv _putenv // Windows deprecates putenv
#endif

int sc_main( int, char*[] )
{
    // prepare environment variable (takes a char*)
    putenv(const_cast<char*>("SC_SIGNAL_WRITE_CHECK=CONFLICT"));
    // and reset simulation context to pick it up (non-standard)
    sc_core::sc_get_curr_simcontext()->reset();

    // report multiple writer errors as warnings
    sc_core::sc_report_handler::set_actions(
        "sc_signal<T> cannot have more than one driver"
      , sc_core::SC_DEFAULT_WARNING_ACTIONS
    );

    dut top("dut");
    sc_core::sc_start();
    std::cout << "\nProgram completed" << std::endl;
    return 0;
}
