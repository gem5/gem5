#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 20) sc_report_handler::stop_after(SC_FATAL,-1) should NOT call sc_stop on 1st fatal error

static bool global_flag_1 = false;
static bool global_flag_2 = false;
static bool global_flag_3 = false;

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }
  void T()
  {
    SC_REPORT_FATAL("/JA", "A bad thing has happened");
    wait(1, SC_NS);
    sc_assert(sc_report_handler::get_count(SC_FATAL) == 1);
    global_flag_1 = true;

    sc_report_handler::stop_after(SC_FATAL, 0);
    SC_REPORT_FATAL("/JA", "A bad thing has happened");
    wait(1, SC_NS);
    sc_assert(sc_report_handler::get_count(SC_FATAL) == 2);
    global_flag_2 = true;

    sc_report_handler::stop_after(SC_FATAL, -1);
    SC_REPORT_FATAL("/JA", "A bad thing has happened");
    wait(1, SC_NS);
    sc_assert(sc_report_handler::get_count(SC_FATAL) == 3);
    global_flag_3 = true;
  }
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
  cout << "Should be silent except for 3 fatal errors..." << endl;

  sc_report_handler::set_actions(SC_FATAL, SC_DISPLAY);

  Top top("top");
  sc_start();

  sc_assert(global_flag_1);
  sc_assert(global_flag_2);
  sc_assert(global_flag_3);
  sc_assert(sc_report_handler::get_count(SC_INFO) == 0);
  sc_assert(sc_report_handler::get_count(SC_WARNING) == 0);
  sc_assert(sc_report_handler::get_count(SC_ERROR) == 0);
  sc_assert(sc_report_handler::get_count(SC_FATAL) == 3);

  cout << endl << "Success" << endl;
  return 0;
}
