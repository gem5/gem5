
// sc_start with event starvation policy

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

  sc_event ev;

  void T()
  {
    ev.notify(150, SC_NS);
  }
};

int sc_main(int argc, char* argv[])
{
  Top top("top");

  sc_assert( sc_get_status() == SC_ELABORATION );
  sc_assert( sc_time_stamp() == SC_ZERO_TIME );
  sc_start(100, SC_NS);
  sc_assert( sc_time_stamp() == sc_time(100, SC_NS) );

  sc_start(10, SC_NS, SC_RUN_TO_TIME);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(10, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(110, SC_NS) );

  sc_start(80, SC_NS, SC_EXIT_ON_STARVATION);
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );  // FAILS - time = 200 NS

  sc_start();
  sc_assert( sc_time_stamp() == sc_time(150, SC_NS) );
  sc_assert( sc_get_status() == SC_PAUSED );

  cout << endl << "Success" << endl;
  return 0;
}
