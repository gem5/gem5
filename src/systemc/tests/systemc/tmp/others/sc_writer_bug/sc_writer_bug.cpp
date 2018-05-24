// sc_writer_policy template argument of class sc_signal

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>

using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;
using std::string;


struct M: sc_module
{
  sc_inout<bool> port;

  sc_time delay;

  M(sc_module_name _name, sc_time _delay)
  : port("port")
  , delay(_delay)
  {
    SC_THREAD(T);
  }

  void T()
  {
	wait(delay);
	port.write(true);
	cout << "port written in " << name() << " at " << sc_time_stamp() 
	     << endl;
        wait(sc_time(1, SC_NS));
  }

  SC_HAS_PROCESS(M);
};

struct Top: sc_module
{
  M *m1;
  M *m2;

  sc_signal<bool,SC_MANY_WRITERS> multi_sig_1;

  Top(sc_module_name _name)
  : multi_sig_1("multi_sig_1")
  {
    m1 = new M("m1", sc_time(1, SC_PS));
    m2 = new M("m2", sc_time(2, SC_PS));

    m1->port.bind(multi_sig_1);
    m2->port.bind(multi_sig_1);

    multi_sig_1.write(true);  
  }

  SC_HAS_PROCESS(Top);
};


int sc_main(int argc, char* argv[])
{
  Top top("top");
  sc_start(5,SC_PS);

  cout << endl << "Success" << endl;
  return 0;
}

