// Reset and kill a thread process, including nested kills

#include <systemc>

using namespace sc_core;
using std::cout;
using std::endl;

struct M3: sc_module
{
  M3(sc_module_name _name)
  {
    SC_THREAD(ticker);
      k = sc_get_current_process_handle();

    SC_THREAD(calling);

    SC_THREAD(target);
      t = sc_get_current_process_handle();

    killing_over = false;
  }

  sc_process_handle t, k;
  sc_event ev;
  int count;
  bool killing_over;

  void ticker()
  {
    for (;;)
    {
      try {
        wait(10, SC_NS);
        ev.notify();
      }
      catch (const sc_unwind_exception& ex) {
        // ticker process killed by target
        cout << "sc_unwind_exception caught by ticker" << endl;
        sc_assert( !ex.is_reset() );
        sc_assert( count == 1 );
        sc_assert( !killing_over );
        throw ex;
      }
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
    sc_assert( !killing_over );
    killing_over = true;

    // Target killed immediately at time 35 NS
    sc_assert( t.terminated() ); // FAILS IN PRESENCE OF k.kill(); on line 96
    sc_assert( k.terminated() );

    sc_stop();
  }

  void target()
  {
    cout << "Target called/reset at " << sc_time_stamp() << endl;
    count = 0;
    for (;;)
    {
      try {
        wait(ev);
        cout << "Target awoke at " << sc_time_stamp() << endl;
        ++count;
      }
      catch (const sc_unwind_exception& ex) {
        cout << "sc_unwind_exception caught by target" << endl;
        if (count == 2)
          sc_assert( ex.is_reset() );
        else if (count == 1)
        {
          sc_assert( !ex.is_reset() );
          sc_assert( !killing_over );
          k.kill();
        }
        else
          sc_assert( false );
        throw ex;
      }
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

