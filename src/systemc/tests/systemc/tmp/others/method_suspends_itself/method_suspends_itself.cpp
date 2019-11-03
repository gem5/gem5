#define SC_INCLUDE_DYNAMIC_PROCESSES

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
    suspend_target = false;
    resume_target = false;
  }
  
  sc_process_handle t;
  sc_event ev;
  bool suspend_target;
  bool resume_target;

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
    // Target runs at 10 NS
    
    suspend_target = true;
    wait(10, SC_NS);
    // Target runs at 20 NS and suspends itself
    
    wait(10, SC_NS);
    // Target does not run at 30 NS
    
    suspend_target = false;
    t.resume();
    // Target runs at 35 NS
    
    wait(10, SC_NS);
    // Target runs at 40 NS  

    suspend_target = true;
    resume_target = true;
    wait(10, SC_NS);
    // Target runs at 50 NS  
    
    sc_stop();
  }

  void target()
  {
    cout << "Target called at " << sc_time_stamp() << endl;
    if (suspend_target)
      t.suspend();
    if (resume_target)
    {
      t.resume();
      suspend_target = false;
    }
  }
  
  SC_HAS_PROCESS(M5);
};

int sc_main(int argc, char* argv[])
{
  M5 m("m");
  
  sc_core::sc_allow_process_control_corners = true;
  sc_start();
  
  return 0;
}
  
