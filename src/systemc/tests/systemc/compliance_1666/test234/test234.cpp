#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 34) event finder on multiport

struct i_f: virtual sc_interface
{
  virtual const sc_event& event() const = 0;
};

struct Chan: i_f, sc_object
{
  virtual const sc_event& event() const { return ev; }
  sc_event ev;
};

struct SCPort: sc_port<i_f,0>
{
  sc_event_finder& find_event() const
  {
    return *new sc_event_finder_t<i_f>( *this, &i_f::event );
  }
};

SC_MODULE(M)
{
  SCPort mp;
  bool flag, flag2;

  SC_CTOR(M)
    : flag(false), flag2(false)
  {
    SC_THREAD(T);
    sensitive << mp.find_event();
  }
  void T()
  {
    wait();
    sc_assert(sc_time_stamp() == sc_time(1, SC_NS));
    wait();
    sc_assert(sc_time_stamp() == sc_time(11, SC_NS));
    wait();
    sc_assert(sc_time_stamp() == sc_time(111, SC_NS));
    flag = true;
    wait();
    flag = false;
  }
  void end_of_elaboration()
  {
    SC_THREAD(T2);
    for (int i = 0; i < mp.size(); i++)
      sensitive << mp[i]->event();
  }
  void T2()
  {
    wait();
    sc_assert(sc_time_stamp() == sc_time(1, SC_NS));
    wait();
    sc_assert(sc_time_stamp() == sc_time(11, SC_NS));
    wait();
    sc_assert(sc_time_stamp() == sc_time(111, SC_NS));
    flag2 = true;
    wait();
    flag2 = false;
  }
  void end_of_simulation()
  {
    sc_assert( flag );
    sc_assert( flag2 );
  }
};

SC_MODULE(Top)
{
  M *m;
  Chan chan1, chan2, chan3;
  SC_CTOR(Top)
  {
    m = new M("m");
    m->mp(chan1);
    m->mp(chan2);
    m->mp(chan3);
    SC_THREAD(T);
  }
  void T()
  {
    wait(1, SC_NS);
    chan1.ev.notify();
    wait(10, SC_NS);
    chan2.ev.notify();
    wait(100, SC_NS);
    chan3.ev.notify();
    wait(1, SC_NS);
    sc_stop();
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
