#include <systemc.h>
sc_trace_file* sc_tf;

class Mod : public sc_module
{
 public:
  sc_in_clk clk;
  sc_in<sc_uint<37> >     a;

  SC_HAS_PROCESS(Mod);
  void foo()
  {
    cout << sc_time_stamp() << "\n";
    cout << "  a = " << a << "\n";
    cout << "\n";

    return;

  }  // foo()

  Mod(const sc_module_name& name) : sc_module(name), a("a")
  {
    SC_METHOD(foo);
    sensitive << clk.pos();
    dont_initialize();

    sc_trace(sc_tf, a, a.name());
  }

};  // class Mod


int sc_main(int argc, char* argv[])
{
  sc_clock clk("clk", 50, SC_NS, 0.5, 0, SC_NS);
  sc_signal<sc_uint<37> > a;

  sc_tf = sc_create_vcd_trace_file("test13");

  Mod mod("mod");

  mod.clk(clk);
  mod.a(a);

  sc_trace(sc_tf, clk, clk.name());

  sc_start(50, SC_NS);
  a = 12;
  sc_start(50, SC_NS);

  return 0;

}  // sc_main()
