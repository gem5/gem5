#include <systemc.h>

// Multiple tests for Annex D.1 and D.2

struct i_f: virtual sc_interface
{
};

struct Chan: i_f, sc_object
{
};

struct Port: sc_port<i_f>
{
  Chan chan;
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

SC_MODULE(M)
{
  Port port;

  SC_CTOR(M)
  {
    SC_THREAD(T);
    std::string s1 = std::string(sc_get_current_process_handle().name());
    sc_assert(s1.substr(0,7) == "top.m.T");
    check_form_of_suffix(s1.substr(7));

    SC_THREAD(T);
    std::string s2 = std::string(sc_get_current_process_handle().name());
    sc_assert(s2.substr(0,7) == "top.m.T");
    check_form_of_suffix(s2.substr(7));

    SC_THREAD(T);
    std::string s3 = std::string(sc_get_current_process_handle().name());
    sc_assert(s3.substr(0,7) == "top.m.T");
    check_form_of_suffix(s3.substr(7));

    sc_assert(s1 != s2);
    sc_assert(s2 != s3);
    sc_assert(s3 != s1);

    SC_THREAD(R);
  }

  void end_of_elaboration()
  {
    sc_assert(port.get_parent_object() == this);
    sc_assert(port.chan.get_parent_object() == this);
    sc_assert(get_child_objects().size() == 6);
  }

  void T()
  {
    int i_count = sc_report_handler::get_count(SC_INFO);
    int w_count = sc_report_handler::get_count(SC_WARNING);
    int e_count = sc_report_handler::get_count(SC_ERROR);
    int f_count = sc_report_handler::get_count(SC_FATAL);

    sc_report_handler::set_actions(SC_INFO,    SC_DO_NOTHING);
    sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);
    sc_report_handler::set_actions(SC_ERROR,   SC_DO_NOTHING);
    sc_report_handler::set_actions(SC_FATAL,   SC_DO_NOTHING);

    SC_REPORT_INFO("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 1);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count);

    SC_REPORT_WARNING("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 1);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 1);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count);

    SC_REPORT_ERROR("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 1);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 1);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 1);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count);

    SC_REPORT_FATAL("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 1);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 1);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 1);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count + 1);

    sc_report_handler::set_actions(SC_INFO,    SC_DISPLAY);
    sc_report_handler::set_actions(SC_WARNING, SC_DISPLAY);
    sc_report_handler::set_actions(SC_ERROR,   SC_DISPLAY);
    sc_report_handler::set_actions(SC_FATAL,   SC_DISPLAY);

    SC_REPORT_INFO("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 2);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 1);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 1);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count + 1);

    SC_REPORT_WARNING("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 2);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 2);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 1);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count + 1);

    SC_REPORT_ERROR("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 2);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 2);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 2);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count + 1);

    SC_REPORT_FATAL("type", "msg");

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count + 2);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count + 2);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 2);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count + 2);
  }
  void R()
  {
  wait(100, SC_NS);

    int i_count = sc_report_handler::get_count(SC_INFO);
    int w_count = sc_report_handler::get_count(SC_WARNING);
    int e_count = sc_report_handler::get_count(SC_ERROR);
    int f_count = sc_report_handler::get_count(SC_FATAL);

    try {
      SC_REPORT_ERROR("type", "msg");
    }
    catch (sc_report& rpt) {
      sc_assert(rpt.get_severity() == SC_ERROR);
      sc_assert(strcmp(rpt.get_msg_type(), "type") == 0);
      sc_assert(strcmp(rpt.get_msg(), "msg") == 0);
      sc_assert(rpt.get_time() == sc_time(0, SC_NS));
      sc_assert(strcmp(rpt.get_process_name(), "top.m.R") == 0);
    }

    sc_assert(sc_report_handler::get_count(SC_INFO)    == i_count);
    sc_assert(sc_report_handler::get_count(SC_WARNING) == w_count);
    sc_assert(sc_report_handler::get_count(SC_ERROR)   == e_count + 1);
    sc_assert(sc_report_handler::get_count(SC_FATAL)   == f_count);
  }
};

SC_MODULE(Top)
{
  M *m;
  Chan *chan;
  SC_CTOR(Top)
  {
    m = new M("m");
    chan = new Chan;
    m->port.bind(*chan);
  }
  void end_of_elaboration()
  {
    sc_assert(get_child_objects().size() == 2);
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent except for reports ..." << endl;

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
