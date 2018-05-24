#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 3) terminated_event()

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }

  void T()
  {
    sc_process_handle h1 = sc_spawn(sc_bind(&M::proc, this, 2));
    sc_process_handle h2 = sc_spawn(sc_bind(&M::proc, this, 3));
    sc_process_handle h3 = sc_spawn(sc_bind(&M::proc, this, 1));
    sc_assert(sc_time_stamp() == sc_time(0, SC_NS));
    wait(h1.terminated_event() & h2.terminated_event() & h3.terminated_event());
    sc_assert(sc_time_stamp() == sc_time(3, SC_NS));
    if (h1.valid()) sc_assert (h1.terminated());
    if (h2.valid()) sc_assert (h2.terminated());
    if (h3.valid()) sc_assert (h3.terminated());

    h1 = sc_spawn(sc_bind(&M::proc, this, 10));
    h2 = sc_spawn(sc_bind(&M::proc, this, 30));
    h3 = sc_spawn(sc_bind(&M::proc, this, 20));
    sc_assert(sc_time_stamp() == sc_time(3, SC_NS));
    wait(h1.terminated_event() | h2.terminated_event() | h3.terminated_event());
    sc_assert(sc_time_stamp() == sc_time(13, SC_NS));
    sc_assert(h2.valid());
    sc_assert( !h2.terminated() );
    sc_assert(h3.valid());
    sc_assert( !h3.terminated() );

    wait(h2.terminated_event() & h3.terminated_event());
    sc_assert(sc_time_stamp() == sc_time(33, SC_NS));
  }
  void proc(int delay)
  {
    wait(delay * sc_time(1, SC_NS));
  }
};

SC_MODULE(Top)
{
  M *m;
  SC_CTOR(Top)
  {
    m = new M("m");
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent..." << endl;

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
