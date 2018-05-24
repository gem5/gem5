#include <systemc>
#include <cstring>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 33) Process macros in (before_)end_of_elaboration

SC_MODULE(M)
{
  SC_CTOR(M)
  {
    SC_THREAD(T);
  }
  void T()
  {
  }
  void before_end_of_elaboration()
  {
    SC_THREAD(T1);
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.T1") == 0);
  sc_assert(h.proc_kind() == SC_THREAD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == false);

  SC_METHOD(M1);
    h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.M1") == 0);
  sc_assert(h.proc_kind() == SC_METHOD_PROC_);
  sc_assert(h.get_process_object() != 0);
  children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == false);
  }

  void end_of_elaboration()
  {
    SC_THREAD(T2);
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.T2") == 0);
  sc_assert(h.proc_kind() == SC_THREAD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == true);

  SC_METHOD(M2);
    h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.M2") == 0);
  sc_assert(h.proc_kind() == SC_METHOD_PROC_);
  sc_assert(h.get_process_object() != 0);
  children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == true);
  }

  void T1 ()
  {
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.T1") == 0);
  sc_assert(h.proc_kind() == SC_THREAD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == false);
  }
  void M1 ()
  {
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.M1") == 0);
  sc_assert(h.proc_kind() == SC_METHOD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == false);
  }
  void T2 ()
  {
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.T2") == 0);
  sc_assert(h.proc_kind() == SC_THREAD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == true);
  }
  void M2 ()
  {
    sc_process_handle h = sc_get_current_process_handle();
  sc_assert(h.valid() == true);
  sc_assert(strcmp(h.name(), "top.m.M2") == 0);
  sc_assert(h.proc_kind() == SC_METHOD_PROC_);
  sc_assert(h.get_process_object() != 0);
  std::vector<sc_object*> children = h.get_child_objects();
  sc_assert(children.size() == 0);
  sc_assert(h.get_parent_object() == this);
  sc_assert(h.terminated() == false);
  sc_assert(h.dynamic() == true);
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
  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
