#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M: sc_module
{
  M(sc_module_name _name)
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
    // Target runs at time 10 NS due to notification
    
    t.sync_reset_on();
    // Target does not run at time 15 NS 
    
    wait(10, SC_NS);
    // Target is reset at time 20 NS due to notification 
    
    wait(10, SC_NS);
    // Target is reset again at time 30 NS due to notification 
    
    t.sync_reset_off();
    // Target does not run at time 35 NS 
    
    wait(10, SC_NS);
    // Target runs at time 40 NS due to notification
    
    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    for (;;)
    {
      wait(ev);
      cout << "Target awoke at " << sc_time_stamp() << endl;
    }
  }
  
  SC_HAS_PROCESS(M);
};

int sc_main(int argc, char* argv[])
{
  M m("m");
  
  sc_start();
  
  return 0;
}
  
