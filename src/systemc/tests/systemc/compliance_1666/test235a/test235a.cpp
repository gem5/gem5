#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 35) Port policy (correct binding)

SC_MODULE(M)
{
  sc_port<sc_signal_in_if<int>,1,SC_ONE_OR_MORE_BOUND> p1;
  sc_port<sc_signal_in_if<int>,2,SC_ONE_OR_MORE_BOUND> p2;
  sc_port<sc_signal_in_if<int>,2,SC_ONE_OR_MORE_BOUND> p3;
  sc_port<sc_signal_in_if<int>,0,SC_ONE_OR_MORE_BOUND> p4;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p5;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p6;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p7;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p8;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p9;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p10;
  sc_port<sc_signal_in_if<int>,1,SC_ALL_BOUND> p11;
  sc_port<sc_signal_in_if<int>,3,SC_ALL_BOUND> p12;

  sc_port<sc_signal_in_if<int>,2,SC_ALL_BOUND> p1_all_2;
  sc_port<sc_signal_in_if<int>,2,SC_ALL_BOUND> p2_all_2;
  sc_port<sc_signal_in_if<int>,2,SC_ALL_BOUND> p3_all_2;
  sc_port<sc_signal_in_if<int>,5,SC_ALL_BOUND> p4_all_5;
  sc_port<sc_signal_in_if<int>,2,SC_ALL_BOUND> p5_all_2;

  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p6_zero;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p7_one;
  sc_port<sc_signal_in_if<int>,2,SC_ZERO_OR_MORE_BOUND> p8_two;

  SC_CTOR(M)
    : p1("p1"),
      p2("p2"),
      p3("p3"),
      p4("p4"),
      p5("p5"),
      p6("p6"),
      p7("p7"),
      p8("p8"),
      p9("p9"),
      p10("p10"),
      p11("p11"),
      p12("p12"),
      p1_all_2("p1_all_2"),
      p2_all_2("p2_all_2"),
      p3_all_2("p3_all_2"),
      p4_all_5("p4_all_5"),
      p5_all_2("p5_all_2"),
      p6_zero("p6_zero"),
      p7_one("p7_one"),
      p8_two("p8_two")
  {}
  void end_of_elaboration()
  {
    sc_assert(p1.size() == 1);
    sc_assert(p2.size() == 1);
    sc_assert(p3.size() == 2);
    sc_assert(p4.size() == 3);
    sc_assert(p5.size() == 0);
    sc_assert(p6.size() == 1);
    sc_assert(p7.size() == 2);
    sc_assert(p8.size() == 0);
    sc_assert(p9.size() == 1);
    sc_assert(p10.size() == 2);
    sc_assert(p11.size() == 1);
    sc_assert(p12.size() == 3);

    sc_assert(p1_all_2.size() == 2);
    sc_assert(p2_all_2.size() == 2);
    sc_assert(p3_all_2.size() == 2);
    sc_assert(p4_all_5.size() == 5);
    sc_assert(p5_all_2.size() == 2);

    sc_assert(p6_zero.size() == 0);
    sc_assert(p7_one.size() == 1);
    sc_assert(p8_two.size() == 2);
  }
};

SC_MODULE(Top)
{
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p1_twice;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p2_once;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p3_once;
  sc_port<sc_signal_in_if<int>,0,SC_ZERO_OR_MORE_BOUND> p4_unbound;

  M *m;
  sc_signal<int> sig1, sig2, sig3, sig4;

  SC_CTOR(Top)
  {
    m = new M("m");
    m->p1(sig1);

    m->p2(sig1);

    m->p3(sig1);
    m->p3(sig2);

    m->p4(sig1);
    m->p4(sig2);
    m->p4(sig3);

    m->p6(sig3);

    m->p7(sig1);
    m->p7(sig2);

    m->p9(sig1);

    m->p10(sig1);
    m->p10(sig2);

    m->p11(sig1);

    m->p12(sig1);
    m->p12(sig2);
    m->p12(sig3);

    m->p1_all_2(p1_twice);

    m->p2_all_2(p2_once);
    m->p2_all_2(p3_once);

    m->p3_all_2(sig1);
    m->p3_all_2(p2_once);

    m->p4_all_5(p2_once);
    m->p4_all_5(sig1);
    m->p4_all_5(p1_twice);
    m->p4_all_5(p3_once);
    m->p4_all_5(p4_unbound);

    m->p5_all_2(p2_once);
    m->p5_all_2(p4_unbound);

    m->p6_zero(p4_unbound);

    m->p7_one(p2_once);

    m->p8_two(p2_once);
  }
  void before_end_of_elaboration()
  {
    m->p5_all_2(p4_unbound);
    m->p5_all_2(p3_once);
    m->p5_all_2(p4_unbound);

    m->p6_zero(p4_unbound);

    m->p8_two(sig1);
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent..." << endl;

  sc_signal<int> sig1, sig2, sig3, sig4;

  Top top("top");
  top.p1_twice(sig1);
  top.p1_twice(sig2);

  top.p2_once(sig3);
  top.p3_once(sig4);

  sc_start(1, SC_NS);

  cout << endl << "Success" << endl;
  return 0;
}
