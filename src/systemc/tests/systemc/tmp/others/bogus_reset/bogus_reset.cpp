// Bogus reset

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M5: sc_module
{
  M5(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_METHOD(target);
      sensitive << ev;
      dont_initialize();
      t = sc_get_current_process_handle();
  }

  sc_process_handle t;
  sc_event ev;

  void ticker()
  {
    for (;;)
    {
      wait(10, SC_NS);
      ev.notify();
    }
  }

  void calling()
  {
    wait(15, SC_NS);
    // target runs at 10 NS due to notification of ev

    t.reset();
    // target runs at 15 NS due to reset.

    sc_stop();
  }

  void target()
  {
    cout << "Target called at " << sc_time_stamp() << endl;
  }

  SC_HAS_PROCESS(M5);
};

int sc_main(int argc, char* argv[])
{
  M5 m("m");

  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}

