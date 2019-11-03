// sync_reset_on/off

#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M2: sc_module
{
  M2(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_THREAD(target);
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

    t.sync_reset_on();
    wait(10, SC_NS);

    t.sync_reset_off();
    wait(10, SC_NS);

    t.reset();
    wait(SC_ZERO_TIME);
    
    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;

    for (;;)
    {
      try {
        wait(ev);
        cout << "Target awoke at " << sc_time_stamp() << endl;
      }
      catch (const sc_unwind_exception& ex) {
        cout << "Unwinding at " << sc_time_stamp() << endl;
        sc_assert( t.is_unwinding() );
        sc_assert( sc_is_unwinding() );
        throw ex;
      }
    }
  }
  
  SC_HAS_PROCESS(M2);
};

int sc_main(int argc, char* argv[])
{
  M2 m("m");
  
  sc_start();
  
  cout << endl << "Success" << endl;
  return 0;
}
  
