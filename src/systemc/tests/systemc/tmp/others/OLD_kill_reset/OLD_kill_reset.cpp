#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M3: sc_module
{
  M3(sc_module_name _name)
  {
    SC_THREAD(ticker);
    SC_THREAD(calling);
    SC_THREAD(target);
      t = sc_get_current_process_handle();
  }
  
  sc_process_handle t;
  sc_event ev;
  int count;

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
    sc_assert( count == 1 );
 
    wait(10, SC_NS);
    // Target runs again at time 20 NS due to notification
    sc_assert( count == 2 );
    
    t.reset();
    // Target reset immediately at time 25 NS
    sc_assert( count == 0 );
 
    wait(10, SC_NS);
    // Target runs again at time 30 NS due to notification
    sc_assert( count == 1 );
    
    t.kill();
    // Target killed immediately at time 35 NS
    sc_assert( t.terminated() );
      
    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    count = 0;
    for (;;)
    {
      wait(ev);
      cout << "Target awoke at " << sc_time_stamp() << endl;
      ++count;
    }
  }
  
  SC_HAS_PROCESS(M3);
};

int sc_main(int argc, char* argv[])
{
  M3 m("m");
  
  sc_start();
  
  return 0;
}
  
