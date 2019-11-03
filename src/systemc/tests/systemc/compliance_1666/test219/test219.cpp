#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 19) Call to sc_spawn in what would have been the final update phase...

sc_event ev;

struct Prim: sc_prim_channel, sc_interface
{
  Prim() : count(0) {}

  void write() {
    request_update();
  }
  void update() {
    sc_spawn(sc_bind(&Prim::spawned_proc, this));
  }
  void spawned_proc()
  {
    ++ count;
    ev.notify(5, SC_NS);
  }
  const sc_event& default_event() const { return ev; }
  int count;
};

SC_MODULE(M)
{
  Prim prim;
  SC_CTOR(M)
    : ME_count(0)
  {
    SC_THREAD(T);
    SC_METHOD(ME);
      sensitive << prim;
      dont_initialize();
  }
  void T()
  {
    wait(10, SC_NS);
    prim.write();
    sc_assert(sc_time_stamp() == sc_time(10, SC_NS));
  }
  void ME()
  {
    ++ ME_count;
    sc_assert(sc_time_stamp() == sc_time(15, SC_NS));
  }
  int ME_count;
};

struct Top: sc_module
{
  M *m;
  Top(sc_module_name)
  {
    m = new M("m");
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent..." << endl;

  Top top("top");
  sc_start();

  sc_assert(top.m->prim.count == 1);
  sc_assert(top.m->ME_count == 1);


  cout << endl << "Success" << endl;
  return 0;
}
