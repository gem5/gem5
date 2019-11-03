#include <systemc>

using sc_core::sc_delta_count;
using sc_core::sc_is_running;
using sc_core::SC_ZERO_TIME;
using sc_core::sc_start;
using sc_core::sc_event;
using sc_core::SC_NS;
using std::cout;
using std::endl;

// D.1 6) sc_delta_count()

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }
  void T()
  {
    cout << "T() " << sc_is_running() << "  " << sc_delta_count() << endl;
    wait(10, SC_NS);
    cout << "T() " << sc_is_running() << "  " << sc_delta_count() << endl;
    wait(10, SC_NS);
    cout << "T() " << sc_is_running() << "  " << sc_delta_count() << endl;
    wait(10, SC_NS);
    cout << "T() " << sc_is_running() << "  " << sc_delta_count() << endl;
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
  sc_assert(sc_delta_count() == 0);

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
