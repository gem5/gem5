#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 9) sc_object registered and named

struct i_f: virtual sc_interface
{
};

struct Chan: i_f, sc_object
{
  Chan() {}
  Chan(const char* _n): sc_object(_n) {}
};

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }
  void T()
  {
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

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent..." << endl;
  sc_report_handler::set_actions(SC_WARNING, SC_DO_NOTHING);

  Chan ch1(""); // object
  std::string s1 = std::string(ch1.name());
  sc_assert(s1.substr(0,6) == "object");
  check_form_of_suffix(s1.substr(6));

  Chan ch2;  // object_0
  const std::string s2 = std::string(ch2.name());
  sc_assert(s2.substr(0,6) == "object");
  check_form_of_suffix(s2.substr(6));
  sc_assert(s2 != s1);

  Chan ch3(s2.c_str()); // object_0_0 or object_1
  std::string s3 = std::string(ch3.name());
  sc_assert(s3.substr(0,6) == "object");
  check_form_of_suffix(s3.substr(6));
  sc_assert(s3 != s1);
  sc_assert(s3 != s2);

  Chan ch4("object_2");
  std::string s4 = std::string(ch4.name());
  sc_assert(s4.substr(0,6) == "object");
  check_form_of_suffix(s4.substr(6));
  sc_assert(s4 != s1);
  sc_assert(s4 != s2);
  sc_assert(s4 != s3);

  Chan ch5;
  std::string s5 = std::string(ch5.name());
  sc_assert(s5.substr(0,6) == "object");
  check_form_of_suffix(s5.substr(6));
  sc_assert(s5 != s1);
  sc_assert(s5 != s2);
  sc_assert(s5 != s3);
  sc_assert(s5 != s4);

  Chan ch6("");
  std::string s6 = std::string(ch6.name());
  sc_assert(s6.substr(0,6) == "object");
  check_form_of_suffix(s6.substr(6));
  sc_assert(s6 != s1);
  sc_assert(s6 != s2);
  sc_assert(s6 != s3);
  sc_assert(s6 != s4);
  sc_assert(s6 != s5);

  sc_signal<int> sig7("signal_0");
  std::string s7 = std::string(sig7.name());
  sc_assert(s7 == "signal_0");
  sc_assert(s7 != s1);
  sc_assert(s7 != s2);
  sc_assert(s7 != s3);
  sc_assert(s7 != s4);
  sc_assert(s7 != s5);
  sc_assert(s7 != s6);

  sc_signal<int> sig8;
  std::string s8 = std::string(sig8.name());
  sc_assert(s8.substr(0,6) == "signal");
  sc_assert(s8.size() > 6);
  check_form_of_suffix(s8.substr(6));
  sc_assert(s8 != s1);
  sc_assert(s8 != s2);
  sc_assert(s8 != s3);
  sc_assert(s8 != s4);
  sc_assert(s8 != s5);
  sc_assert(s8 != s6);
  sc_assert(s8 != s7);

  std::vector<sc_object*> children = sc_get_top_level_objects();
  sc_assert (children.size() == 8);

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
