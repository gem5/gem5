#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M4: sc_module
{
  M4(sc_module_name _name)
  {
    SC_THREAD(calling);
    SC_THREAD(target);
      t = sc_get_current_process_handle();
  }
  
  sc_process_handle t;
  sc_event ev;
  int count;

  void calling()
  {
  
    t.sync_reset_on(); 
    wait(10, SC_NS);

    t.suspend();
    wait(10, SC_NS);

    t.disable();
    wait(10, SC_NS);

    t.enable();
    ev.notify();
    wait(10, SC_NS);  // !!!!!! target is RESET WHILE STILL SUSPENDED !!!!!!

    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    count = 0;
    for (;;)
    {
      wait(ev);
      cout << "Target awoke at " << sc_time_stamp() << " count = " << count << endl;
      ++count;
    }
  }
  
  SC_HAS_PROCESS(M4);
};

int sc_main(int argc, char* argv[])
{
  M4 m("m");
  
  sc_core::sc_allow_process_control_corners = true;
  sc_start();
  
  return 0;
}
  
