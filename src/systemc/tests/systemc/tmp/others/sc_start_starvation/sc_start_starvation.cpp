
// sc_start with event starvation policy

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using std::cout;
using std::endl;

SC_MODULE(Top)
{
  SC_CTOR(Top)
  {
    SC_THREAD(T);
  }

  sc_event ev2;

  void T()
  {
    sc_assert( sc_get_status() == SC_RUNNING );
    ev2.notify(150, SC_NS);

    //wait(ev2);  // Inserting this line makes the test pass
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");

  sc_event ev;
  ev.notify(250, SC_NS);

  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_time_stamp() == SC_ZERO_TIME );
  sc_start(100, SC_NS);
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_time_stamp() == sc_time(100, SC_NS) );

  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(10, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(80, SC_NS, SC_EXIT_ON_STARVATION);

  cout << "sc_time_stamp() = " << sc_time_stamp() << endl;
  cout << "sc_pending_activity_at_future_time() = " << sc_pending_activity_at_future_time() << endl;
  cout << "sc_time_to_pending_activity() = " << sc_time_to_pending_activity() << endl;

  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );  // FAILS. Does not see ev2

  sc_start(50, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );

  sc_start(50, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(200, SC_NS) );

  sc_start();
  sc_assert( sc_get_status() == SC_PAUSED );
  sc_assert( sc_time_stamp() == sc_time(250, SC_NS) );

  ev.notify(SC_ZERO_TIME);
  sc_start();
  sc_assert( sc_time_stamp() == sc_time(250, SC_NS) );

  ev.notify(10, SC_NS);
  sc_start();
  sc_assert( sc_time_stamp() == sc_time(260, SC_NS) );

  ev.notify(10, SC_NS);
  sc_start(sc_time(100, SC_NS), SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(270, SC_NS) );

  ev.notify(10, SC_NS);
  sc_start(sc_time(100, SC_NS)); // SC_RUN_TO_TIME
  sc_assert( sc_time_stamp() == sc_time(370, SC_NS) );
  sc_assert( sc_get_status() == SC_PAUSED );

  cout << endl << "Success" << endl;
  return 0;
}
