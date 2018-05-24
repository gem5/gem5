#include <systemc>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 7) "event_queue"

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
  sc_event_queue eq;
  M *m;
  Top(sc_module_name)
  {
    m = new M("m");

    std::string s = eq.basename();
    sc_assert (s.substr(0,11) == "event_queue");
    sc_assert (s.size() > 11);
    check_form_of_suffix(s.substr(11));

    s = eq.name();
    sc_assert (s.substr(0,15) == "top.event_queue");
    sc_assert (s.size() > 15);
    check_form_of_suffix(s.substr(15));

    sc_assert (std::string(eq.kind()) == "sc_event_queue");
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
