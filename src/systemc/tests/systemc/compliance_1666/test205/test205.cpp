#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using sc_core::wait;
using std::cout;
using std::endl;

// 5) wait( int ) for SC_THREAD, primitives and global

void global()
{
  wait();
  sc_assert(sc_time_stamp() == sc_time(0, SC_NS));
  wait(3);
  sc_assert(sc_time_stamp() == sc_time(3, SC_NS));
}

struct Prim: sc_prim_channel
{
  void method()
  {
    wait();
    sc_assert(sc_time_stamp() == sc_time(4, SC_NS));
    wait(3);
    sc_assert(sc_time_stamp() == sc_time(7, SC_NS));
  }
};

SC_MODULE(M)
{
  sc_in_clk clk;
  Prim prim;
  SC_CTOR(M)
  {
    SC_THREAD(T);
    sensitive << clk.pos();
  }
  void T()
  {
    global();
    prim.method();
    wait();
    sc_assert(sc_time_stamp() == sc_time(8, SC_NS));
    wait(3);
    sc_assert(sc_time_stamp() == sc_time(11, SC_NS));
    sc_stop();
  }
};

struct Top: sc_module
{
  M *m;
  sc_clock clk;
  Top(sc_module_name)
  {
    m = new M("m");
    m->clk.bind(clk);
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
