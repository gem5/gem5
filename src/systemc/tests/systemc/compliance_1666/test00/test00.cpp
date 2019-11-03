
// General tests combining features of SystemC 2.1 and 1666


#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
using sc_core::sc_interface;
using sc_core::sc_object;
using sc_core::sc_port;
using sc_core::sc_export;
using sc_core::sc_module;
using sc_core::sc_module_name;
using sc_core::sc_in;
using sc_core::sc_in_clk;
using sc_core::sc_signal;
using sc_core::sc_prim_channel;
using sc_core::sc_event;
using sc_core::sc_delta_count;
using sc_core::sc_is_running;
using sc_core::sc_get_top_level_objects;
using sc_core::sc_find_object;
using sc_core::sc_report;
using sc_core::sc_report_handler;
using sc_core::sc_actions;
using sc_core::SC_INFO;
using sc_core::SC_UNSPECIFIED;
using sc_core::SC_THROW;
using sc_core::sc_spawn;
using sc_core::sc_process_handle;
using sc_core::sc_get_current_process_handle;
using sc_core::sc_time;
using sc_core::sc_time_stamp;
using sc_core::SC_ZERO_TIME;
using sc_core::SC_PS;
using sc_core::SC_NS;
using sc_core::SC_US;
using sc_core::sc_start_of_simulation_invoked;
using sc_core::sc_end_of_simulation_invoked;
using sc_core::sc_start;
using sc_core::sc_stop;
using sc_core::sc_copyright;
using sc_core::sc_version;
using sc_core::sc_release;
using sc_dt::sc_abs;
using sc_dt::sc_max;
using sc_dt::sc_min;
using sc_dt::sc_int;
using sc_dt::sc_bigint;
using std::cout;
using std::endl;

#if defined(IEEE_1666_CPLUSPLUS) && IEEE_1666_CPLUSPLUS >= 201103L
// dynamic process macros have been moved to functions on C++11
using sc_core::sc_bind;
using sc_core::sc_ref;
using sc_core::sc_cref;
#endif // IEEE_1666_CPLUSPLUS

struct i_f: virtual sc_interface
{
  virtual void write() = 0;
};

void check_form_of_suffix(std::string s)
{
  std::string charset = "0123456789";
  while (!s.empty())
  {
    sc_assert(s[0] == '_');
  s = s.substr(1);
  sc_assert(!s.empty());
  do
  {
    sc_assert(charset.find(s[0]) < charset.size());
    s = s.substr(1);
  } while (!s.empty() && (s[0] != '_'));
  }
}

struct Chan: i_f, sc_object
{
  virtual void write()
  {
    sc_assert(std::string(name()).substr(0,17) == "top.m_dest.object");
    check_form_of_suffix(std::string(name()).substr(17));
  }
};

struct M_src: sc_core::sc_behavior
{
  sc_port<i_f>* p;
  SC_HAS_PROCESS(M_src);

  M_src(sc_module_name _name)
  {
    p = new sc_port<i_f>;

    SC_THREAD(T);
  }
  void T()
  {
    (*p)->write();
  }
};

struct M_dest: sc_core::sc_channel
{
  sc_export<i_f>* xp;
  SC_HAS_PROCESS(M_dest);

  M_dest(sc_module_name _name)
  {
    xp = new sc_export<i_f>;
    Chan* ch = new Chan;
    xp->bind(*ch);
  }
};

void f()
{
  ::sc_core::wait(33, SC_PS);
}

struct Prim: sc_prim_channel
{
  Prim() : cref_arg(1) {}

  void write() { request_update(); }

  void update()
  {
    sc_spawn(sc_bind(&Prim::proc, this));
  }

  void proc()
  {
    ev.notify(SC_ZERO_TIME);
    wait(ev);
    cout << "Prim::Proc spawned and resumed" << endl;
    SC_FORK
      sc_spawn(sc_bind(&Prim::Th, this, 1)),
      sc_spawn(sc_bind(&Prim::Th, this, 2))
    SC_JOIN
    sc_spawn(sc_bind(&Prim::Th_ref, this, sc_ref(ref_arg)));
    sc_spawn(sc_bind(&Prim::Th_cref, this, sc_cref(cref_arg)));
    wait(1, SC_NS);
    sc_assert(ref_arg == 99);
  }
  sc_event ev;
  void Th(int i)
  {
    cout << "Th::i=" << i << endl;
  }
  void Th_ref(int& arg)
  {
    arg = 99;
    sc_time t(sc_time_stamp());
    sc_process_handle h = sc_spawn(&f);
    wait(h.terminated_event());
    sc_assert(sc_time_stamp() == t + sc_time(33, SC_PS));
  }
  void Th_cref(const int& arg)
  {
  }
  int ref_arg;
  const int cref_arg;
};

SC_MODULE(M)
{
  Prim prim;
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }

  void T()
  {
    wait(10, SC_NS);
    try {
      sc_report_handler::set_actions("msg_type", SC_INFO, SC_THROW);
      SC_REPORT_INFO("msg_type", "msg");
    }
    catch (sc_report& rpt) {
      cout << "Caught rpt " << rpt.what() << endl;
      sc_assert(rpt.get_severity() == SC_INFO);
      sc_assert(std::string(rpt.get_msg_type()) == "msg_type");
      sc_assert(std::string(rpt.get_msg()) == "msg");
      sc_assert(rpt.get_time() == sc_time(10, SC_NS));
      sc_assert(std::string(rpt.get_process_name()) == "top.m.T");

      sc_assert(sc_report_handler::get_count(SC_INFO) == 1);
      sc_report rpt2 = rpt; // Copy constructor

      prim.write();

      sc_spawn(sc_bind(&M::proc, this, &rpt2));
      wait(100, SC_NS);
    }
  }
  void proc(sc_report* rpt)
  {
    sc_assert(rpt->get_severity() == SC_INFO);
    sc_assert(std::string(rpt->get_msg_type()) == "msg_type");
    sc_assert(std::string(rpt->get_msg()) == "msg");
    sc_assert(rpt->get_time() == sc_time(10, SC_NS));
    sc_assert(std::string(rpt->get_process_name()) == "top.m.T");
    rpt->get_file_name();
    rpt->get_line_number();
  }
};

struct CM: sc_module
{
  sc_in_clk clk;
  sc_in<bool> reset;

  SC_HAS_PROCESS(CM);

  CM(sc_module_name _name)
  : first(true)
  {
    SC_CTHREAD(CT, clk.pos());
      reset_signal_is(reset, true);

    sc_assert(sc_start_of_simulation_invoked() == false);
    sc_assert(sc_end_of_simulation_invoked() == false);
    sc_assert(sc_is_running() == false);
  }

  bool first;

  void CT()
  {
    if (first)
      sc_assert(sc_time_stamp() == sc_time(5, SC_US));
    else
      sc_assert(sc_time_stamp() == sc_time(30, SC_US));
    first = false;

    sc_assert(sc_start_of_simulation_invoked() == true);
    sc_assert(sc_end_of_simulation_invoked() == false);
    sc_assert(sc_is_running() == true);

    while(1)
    {
      wait();
      sc_assert(sc_time_stamp() == sc_time(15, SC_US));
      wait();
      sc_assert(sc_time_stamp() == sc_time(25, SC_US));
      wait();
      sc_assert(false);
    }
  }

  void start_of_simulation()
  {
    sc_assert(sc_start_of_simulation_invoked() == false);
    sc_assert(sc_end_of_simulation_invoked() == false);
    sc_assert(sc_is_running() == false);
  }

  void end_of_simulation()
  {
    sc_assert(sc_start_of_simulation_invoked() == true);
    sc_assert(sc_end_of_simulation_invoked() == false);
    sc_assert(sc_is_running() == false);
  }
};

SC_MODULE(Top)
{
  M *m;
  CM *cm;

  M_src m_src;
  M_dest m_dest;

  sc_signal<bool> clk, reset;

  SC_CTOR(Top)
  : m_src("m_src"), m_dest("m_dest")
  {
    m = new M("m");
    cm = new CM("cm");
    cm->clk(clk);
    cm->reset(reset);

    m_src.p->bind( *(m_dest.xp) );  // Port-export binding

    SC_THREAD(T);
    SC_THREAD(T2);
  }
  void T()
  {
    clk = false;
    reset = true;
    wait(5, SC_US);
    clk = true;
    wait(5, SC_US);
    clk = false;
    reset = false;
    wait(5, SC_US);
    clk = true;
    wait(5, SC_US);
    clk = false;
    wait(5, SC_US);
    clk = true;
    wait(5, SC_US);
    clk = false;
    reset = true;
    wait(10, SC_US);
  }
  void T2()
  {
    sc_assert(sc_min(-1,1) == -1);
    sc_assert(sc_max(-1,1) == 1);
    sc_assert(sc_abs(-1) == 1);

    sc_assert(sc_min(sc_int<8>(-1),sc_int<8>(1)) == sc_int<8>(-1));
    sc_assert(sc_max(sc_int<8>(-1),sc_int<8>(1)) == sc_int<8>(1));
    sc_assert(sc_abs(sc_int<8>(-1)) == sc_int<8>(1));

    sc_assert(sc_min(sc_bigint<256>(-1),sc_bigint<256>(1)) == sc_bigint<256>(-1));
    sc_assert(sc_max(sc_bigint<256>(-1),sc_bigint<256>(1)) == sc_bigint<256>(1));
    sc_assert(sc_abs(sc_bigint<256>(-1)) == sc_bigint<256>(1));
  }
};

int sc_main(int argc, char* argv[])
{
  sc_assert(sc_delta_count() == 0);
  sc_assert(sc_is_running() == 0);
  sc_assert(sc_time_stamp() == sc_time(SC_ZERO_TIME));
  sc_assert(sc_report_handler::get_count(SC_INFO) == 0);
  sc_assert(sc_report_handler::get_count("foo") == 0);

  sc_actions act1 = sc_report_handler::get_new_action_id();
  sc_actions act2 = sc_report_handler::get_new_action_id();
  while (act2 != SC_UNSPECIFIED)
  {
    sc_assert(act2 != act1);
    act2 = sc_report_handler::get_new_action_id();
  }
  sc_assert(sc_report_handler::get_log_file_name() == 0);
  sc_assert(sc_get_top_level_objects().size() == 0);
  sc_assert(sc_find_object("foo") == 0);
  sc_assert(!sc_get_current_process_handle().valid());

  sc_copyright();
  sc_version();
  std::string release = sc_release();
  int n = release.find('.');
  std::string major = release.substr(0,n);
  release = release.substr(n+1,release.size()-1);
  n = release.find('.');
  std::string minor = release.substr(0,n);
  release = release.substr(n+1,release.size()-1);
  n = release.find('-');
  std::string patch = release.substr(0,n);
  std::string originator = release.substr(n+1,release.size());

  std::string charset =
  "abcdefghijklmnopqrstuvwxyzABSCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-";

  for (unsigned int i = 0; i < major.size(); i++)
    sc_assert(charset.find(major[i]) < charset.size());
  for (unsigned int i = 0; i < minor.size(); i++)
    sc_assert(charset.find(minor[i]) < charset.size());
  for (unsigned int i = 0; i < patch.size(); i++)
    sc_assert(charset.find(patch[i]) < charset.size());
  for (unsigned int i = 0; i < originator.size(); i++)
    sc_assert(charset.find(originator[i]) < charset.size());

  Top top("top");
  sc_start();

  sc_stop();
  sc_assert(sc_end_of_simulation_invoked() == true);

  cout << endl << "Success" << endl;
  return 0;
}
