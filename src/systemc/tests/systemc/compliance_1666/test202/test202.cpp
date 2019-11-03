#include <systemc>
#include <cstring>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 2) sc_clock - start_time and posedge_first (in addition to period and duty_cycle)

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }
  void T()
  {
    wait(10, SC_NS);
    sc_stop();
  }
};

struct Top: sc_module
{
  sc_clock clk;
  M *m;
  Top(sc_module_name)
    : clk("clk", 20, SC_NS, 0.75, 5, SC_NS, false)
  {
    m = new M("m");
    sc_assert(strcmp(clk.name(), "top.clk") == 0);
    sc_assert(strcmp(clk.kind(), "sc_clock") == 0);
    sc_assert(clk.period() == sc_time(20, SC_NS));
    sc_assert(clk.duty_cycle() == 0.75);
    sc_assert(clk.start_time() == sc_time(5, SC_NS));
    sc_assert(clk.posedge_first() == false);
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
