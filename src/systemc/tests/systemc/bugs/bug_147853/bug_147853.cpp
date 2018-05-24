// This tests that sc_clock values are updated during the value update phase
// not during the execution phase of a delta cycle.

#include "systemc.h"


SC_MODULE(Test) {
  sc_in_clk clk;
  sc_event e1;
  sc_time d;

  void main() {
    cerr << sc_time_stamp() <<" " << name() << " clk = " << clk.read() << "\n";
    e1.notify(d);
  }
  SC_CTOR(Test) :d(5,SC_NS) {
    SC_METHOD(main);
    sensitive << e1;
  }
};


int sc_main(int argc,char *argv[]) {
  
  Test t1("t1");
  sc_clock clk("clk",10,SC_NS);
  Test t2("t2");
  
  t1.clk(clk);
  t2.clk(clk);
  
  sc_start(50,SC_NS);
  return 0;
}
