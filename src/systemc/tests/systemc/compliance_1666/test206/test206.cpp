#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
#include <cstring>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 6) sc_get_current_process_handle

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    sc_process_handle h;
    sc_assert (!h.valid());
    h = sc_get_current_process_handle();
    sc_assert (!h.valid());
    SC_THREAD(T);
    h = sc_get_current_process_handle();
    sc_assert (h.valid());
    sc_assert (strcmp(h.name(),"top.m.T") == 0);
    sc_assert (h.proc_kind()==SC_THREAD_PROC_);
    sc_assert (h.dynamic()==false);
    sc_assert (h.terminated()==false);
    sc_assert (h.get_process_object() != 0);
    sc_assert (h.get_parent_object() == this);

    sc_spawn(sc_bind(&M::proc, this), "static_proc");
    h = sc_get_current_process_handle();
    sc_assert (h.valid());
    sc_assert (strcmp(h.name(),"top.m.static_proc") == 0);
    sc_assert (h.proc_kind()==SC_THREAD_PROC_);
    sc_assert (h.dynamic()==false);
    sc_assert (h.terminated()==false);
    sc_assert (h.get_process_object() != 0);
    sc_assert (h.get_parent_object() == this);
  }
  sc_object* obj;
  void T()
  {
    sc_process_handle h;
    sc_assert (!h.valid());
    h = sc_get_current_process_handle();
    sc_assert (h.valid());
    sc_assert (strcmp(h.name(),"top.m.T") == 0);
    sc_assert (h.proc_kind()==SC_THREAD_PROC_);
    sc_assert (h.dynamic()==false);
    sc_assert (h.terminated()==false);
    sc_assert (h.get_process_object() != 0);
    sc_assert (h.get_parent_object() == this);

    obj = h.get_process_object();
    sc_spawn_options opt;
    opt.spawn_method();
    sc_spawn(sc_bind(&M::proc, this), "dynamic_proc", &opt);
    wait(1, SC_NS);
  }
  void proc()
  {
    sc_process_handle h = sc_get_current_process_handle();
    sc_assert (h.valid());
    if (h.dynamic())
    {
      sc_assert (strcmp(h.name(),"top.m.T.dynamic_proc") == 0);
      sc_assert (h.proc_kind()==SC_METHOD_PROC_);
      sc_assert (h.get_parent_object() == obj);
    }
    else
    {
      sc_assert (strcmp(h.name(),"top.m.static_proc") == 0);
      sc_assert (h.proc_kind()==SC_THREAD_PROC_);
      sc_assert (h.get_parent_object() == this);
    }
    sc_assert (h.terminated()==false);
    sc_assert (h.get_process_object() != 0);
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
  cout << "Should be silent..." << endl;
  sc_process_handle h;
  sc_assert (!h.valid());
  h = sc_get_current_process_handle();
  sc_assert (!h.valid());

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
