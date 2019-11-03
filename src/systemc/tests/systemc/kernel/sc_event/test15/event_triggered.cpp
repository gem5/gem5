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

  event_triggered.cpp -- test sc_event::triggered

  Original Author: Philipp A. Hartmann, Intel Corporation - 2017-08-06

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>
#include <iomanip>

#ifdef BENCHMARK
  static const sc_dt::uint64 num_events     = 128;
  static const sc_dt::uint64 num_triggers   = 4;
  static const sc_dt::uint64 num_iterations = 10000000;
# define CHECK(expr) ((void)0)
#else
  static const sc_dt::uint64 num_events     = 16;
  static const sc_dt::uint64 num_triggers   = 4;
  static const sc_dt::uint64 num_iterations = 4;
# define CHECK(expr) sc_assert(expr)
#endif

#ifndef UINT64_C
#if defined(_WIN32) && !defined(__MINGW32__)
# define UINT64_C(v) v ## ui64
#else
# define UINT64_C(v) v ## ULL
#endif
#endif // UINT64_C

using namespace sc_core;

SC_MODULE( module )
{
  sc_vector<sc_event> events;
  sc_event            event_return;

  SC_CTOR( module )
    : events("ev", num_events)
    , m_rng_state()
  {
    SC_THREAD(driver);

    SC_THREAD(consumer_dynamic);
    SC_THREAD(consumer_static); // odd events only
      for(unsigned i = 1; i<events.size(); i+=2)
        sensitive << events[i];
  }
private:

  void driver()
  {
    CHECK( !event_return.triggered() );
    wait(1, SC_NS);
    CHECK( !event_return.triggered() );

    random_notify();
    random_notify();
    wait(event_return);

    random_notify(SC_ZERO_TIME);
    random_notify(SC_ZERO_TIME);
    wait(event_return);

    random_notify(sc_time(1, SC_NS));
    random_notify(sc_time(1, SC_NS));
    wait(event_return);
    wait(2, SC_NS);
    CHECK( !event_return.triggered() );

    for(unsigned i = 0; i < num_triggers; ++i)
      random_notify();
    wait(event_return);
    CHECK( event_return.triggered() );

    for(unsigned i = 0; i < num_triggers; ++i)
      random_notify(SC_ZERO_TIME);
    wait(event_return);
    CHECK( event_return.triggered() );

    for(unsigned iter = 0; iter < num_iterations; ++iter) {
      for(unsigned i = 0; i < num_triggers; ++i) {
        random_notify(sc_time(1, SC_NS));
      }
      wait(event_return);
    }
    CHECK( event_return.triggered() );
  }

  void consumer_dynamic()
  {
    sc_event_or_list events_or; // even events only
    for(unsigned i = 0; i < events.size(); i+=2)
      events_or |= events[i];

    while(true) {
      wait(events_or);
      print_triggered();
      event_return.notify();
    }
  }

  void consumer_static()
  {
    while(true) {
      wait();
      print_triggered();
      event_return.notify();
    }
  }

  void print_triggered()
  {
#ifndef BENCHMARK
    using namespace std;
    cout
      << setw(6) << sc_time_stamp()
      << " (" << sc_delta_count() << "): "
      << sc_get_current_process_handle().name() << ": ";
    for(unsigned i =0; i< events.size(); ++i)
      if (events[i].triggered())
        std::cout << events[i].basename() << " ";
    cout << endl;
#endif
  }

  void random_notify()
    { random_notify(SC_ZERO_TIME, true); }

  void random_notify(const sc_time& t, bool immediate = false)
  {
    sc_event& ev = events[ lcg_rng() % num_events ];
#ifndef BENCHMARK
    using namespace std;
    cout
      << setw(6) << sc_time_stamp()
      << " (" << sc_delta_count() << "): "
      << sc_get_current_process_handle().name() << ": "
      << ev.basename();
    if (immediate) {
      cout << ".notify()";
    } else {
      cout << ".notify(" << t << ")";
    }
    cout << endl;
#endif
    if (immediate) {
      ev.notify();
    } else {
      ev.notify(t);
    }
  }

  unsigned lcg_rng()
  {
    m_rng_state = UINT64_C(2862933555777941757) * m_rng_state
                + UINT64_C(3037000493);
    return ( m_rng_state >> 48 );
  }

  sc_dt::uint64 m_rng_state;
};


int
sc_main( int, char*[] )
{
    module m("m");
    sc_start();
    sc_stop();
    return 0;
}
