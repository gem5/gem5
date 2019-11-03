#define SC_INCLUDE_FX

#include <systemc.h>

// 28) sc_core and sc_dt - check names still visible in <systemc.h>

SC_MODULE(Mod)
{
  SC_CTOR(Mod)
  {
    SC_THREAD(T);
  };
  void T()
  {
    sc_int<4> i;
    sc_uint<4> ui;
    sc_bigint<4> bi;
    sc_biguint<4> bui;
    sc_fixed<4,2> f;
    sc_ufixed<4,2> uf;
    sc_fixed_fast<4,2> ff;
    sc_ufixed_fast<4,2> uff;
    sc_bv<4> bv;
    sc_lv<4> lv;

    sc_int_base ib;
    sc_int_base uib;
    sc_signed s;
    sc_unsigned u;
    sc_fix fi;
    sc_ufix ufi;
    sc_fix_fast ffi;
    sc_ufix_fast uffi;
    sc_logic l;
    sc_bv_base bvb;
    sc_lv_base lvb;

    //sc_value_base vb;
    //sc_fxnum fx;
    //sc_fxnum_fast fxf;
    sc_fxval fxv;
    sc_fxval_fast fxvf;

    sc_assert(sc_abs(-1) == 1);
    sc_assert(sc_max(1, 2) == 2);
    sc_assert(sc_min(1, 2) == 1);
    sc_copyright();
    sc_version();
    sc_release();
  }
  void f(sc_report& rpt)
  {
    sc_severity sev;
    sev = SC_INFO;
    sev = SC_WARNING;
    sev = SC_ERROR;
    sev = SC_FATAL;
    sc_assert (sev == SC_FATAL);

    sc_actions act;
    act = SC_DO_NOTHING;
  }
  void g(sc_exception x) {}
};

struct Top: sc_module
{
  Top(sc_module_name)
  {
   tf = sc_create_vcd_trace_file("vcd");
   tf->set_time_unit(1.0, SC_NS);
   sc_trace(tf, sig, "sig");
   sc_write_comment(tf, "Hello");
  }
  ~Top()
  {
    sc_close_vcd_trace_file(tf);
  }
  sc_trace_file* tf;
  sc_signal<sc_logic> sig;
};

int sc_main(int argc, char* argv[])
{
#if defined(_MSC_VER) && _MSC_VER < 1900
    _set_output_format(_TWO_DIGIT_EXPONENT);
#endif

  cout << "Should be silent except for some renaming warnings..." << endl;

  Top top("top");

  sc_attr_base ab("base");
  sc_attribute<int> at("attr");

  sc_buffer<int> bu;
  sc_signal_resolved sr;
  sc_fifo<int> fi;
  sc_mutex mut;
  sc_semaphore sem(2);
  sc_event_queue eq;

  sc_start(100, SC_NS);

  cout << endl << "Success" << endl;
  return 0;
}
