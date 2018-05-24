#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 35) Port policy (incorrect binding)

SC_MODULE(M)
{
  sc_port<sc_signal_in_if<int>,1,SC_ONE_OR_MORE_BOUND> p1;
  sc_port<sc_signal_in_if<int>,2,SC_ALL_BOUND> p2;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p3;
  sc_port<sc_signal_in_if<int>,1,SC_ONE_OR_MORE_BOUND> p4;
  sc_port<sc_signal_in_if<int>,5,SC_ALL_BOUND> p5;
  sc_port<sc_signal_in_if<int>,5,SC_ALL_BOUND> p6;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p7;

  SC_CTOR(M)
    : p1("p1"),
      p2("p2"),
      p3("p3"),
      p4("p4"),
      p5("p5"),
      p6("p6"),
      p7("p7")
  {}
  void end_of_elaboration()
  {
    sc_assert(p1.size() == 0);
    sc_assert(p2.size() == 1);
    sc_assert(p3.size() == 3);
    sc_assert(p4.size() == 2);
    sc_assert(p5.size() == 6);
    sc_assert(p6.size() == 0);
    sc_assert(p7.size() == 2);
  }
};

SC_MODULE(Top)
{
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p0_unbound;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p1_once;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p2_twice;

  M *m;
  sc_signal<int> sig1, sig2, sig3, sig4;

  SC_CTOR(Top)
  {
    m = new M("m");
    m->p1(p0_unbound);

    m->p2(p1_once);

    m->p3(p1_once);
    m->p3(p2_twice);

    m->p4(p2_twice);

    m->p5(sig1);
    m->p5(p1_once);
    m->p5(sig2);
    m->p5(p2_twice);
    m->p5(sig3);

    m->p7(sig1);
    m->p7(sig1);
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be 7 errors but no aborts ..." << endl;

  sc_report_handler::set_actions(SC_ERROR, SC_DISPLAY);

  sc_signal<int> sig1, sig2, sig3, sig4;

  Top top("top");
  top.p1_once(sig1);

  top.p2_twice(sig2);
  top.p2_twice(sig3);

  sc_start(1, SC_NS);

  sc_assert(sc_report_handler::get_count(SC_ERROR) == 7);

  cout << endl << "Success" << endl;
  return 0;
}
