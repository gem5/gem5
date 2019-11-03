#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
#include <cstring>
using namespace sc_core;
using namespace sc_dt;
using std::cout;
using std::endl;

// 3) Process handle methods, including invalid and terminated process

void invalid_handle_check(sc_process_handle& h)
{

    sc_assert(h.valid() == false);
    sc_assert(strcmp(h.name(), "") == 0);
    sc_assert(h.proc_kind() == SC_NO_PROC_);
    sc_assert(h.get_process_object() == 0);
    std::vector<sc_object*> children = h.get_child_objects();
    sc_assert(children.size() == 0);
    sc_assert(h.get_parent_object() == 0);
    sc_assert(h.terminated() == false);
    sc_assert(h.dynamic() == false);
    sc_assert( !(h == h) );
    sc_assert(h != h);

    cout << "There should be warning 11 messages" << endl;

    h.disable();
    h.enable();
    h.is_unwinding();
    h.kill();
    h.reset();
    h.resume();
    h.suspend();
    h.sync_reset_off();
    h.sync_reset_on();
    h.terminated_event();
    h.throw_it(sc_user(), SC_NO_DESCENDANTS);

    cout << "End of warning messages" << endl;
}

SC_MODULE(M)
{
  sc_in_clk clk;
  SC_CTOR(M)
  {
    sc_process_handle h;
    invalid_handle_check(h);

    SC_THREAD(T);
    SC_CTHREAD(CT, clk.pos());
    SC_METHOD(ME);

    sc_process_handle h3 = sc_spawn(sc_bind(&M::stat_thread, this), "stat_thread");
    sc_spawn_options opt;
    opt.spawn_method();
    sc_process_handle h4 = sc_spawn(sc_bind(&M::stat_method, this), "stat_method", &opt);

    std::vector<sc_object*> children = this->get_child_objects();
    sc_assert(children.size() == 6);
  }
  void T()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h2 = sc_get_current_process_handle();
    sc_assert(h2.valid() == true);
    sc_assert(strcmp(h2.name(), "top.m.T") == 0);
    sc_assert(h2.proc_kind() == SC_THREAD_PROC_);
    sc_assert(h2.get_process_object() != 0);
    std::vector<sc_object*> children2 = h2.get_child_objects();
    sc_assert(children2.size() == 0);
    sc_assert(h2.get_parent_object() == this);
    sc_assert(h2.terminated() == false);
    sc_assert(h2.dynamic() == false);
    sc_assert(h2 == h2);
    sc_assert(h2 != h);

    sc_process_handle h3 = sc_spawn(sc_bind(&M::dyn_thread, this), "dyn_thread");
    wait(1, SC_NS);

    if (h3.valid() == true)
    {
      sc_assert(strcmp(h3.name(), "top.m.T.dyn_thread") == 0);
      sc_assert(h3.proc_kind() == SC_THREAD_PROC_);
      sc_assert(h3.get_process_object() != 0);
      std::vector<sc_object*> children3 = h3.get_child_objects();
      sc_assert(children3.size() == 0);
      sc_assert(h3.get_parent_object() == sc_get_current_process_handle().get_process_object());
      sc_assert(h3.terminated() == true);
      sc_assert(h3.dynamic() == true);
      sc_assert(h3 == h3);
      sc_assert( !(h3 != h3) );
    }

    sc_spawn_options opt;
    opt.spawn_method();
    sc_process_handle h4 = sc_spawn(sc_bind(&M::dyn_method, this), "dyn_method", &opt);
    sc_assert(h4 != h3);

    wait(10, SC_NS);
    sc_stop();
  }

  void stat_thread()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    wait(5, SC_NS);

    sc_process_handle h3 = sc_get_current_process_handle();
    sc_assert(h3.valid() == true);
    sc_assert(strcmp(h3.name(), "top.m.stat_thread") == 0);
    sc_assert(h3.proc_kind() == SC_THREAD_PROC_);
    sc_assert(h3.get_process_object() != 0);
    std::vector<sc_object*> children3 = h3.get_child_objects();
    sc_assert(children3.size() == 0);
    sc_assert(h3.get_parent_object() != 0);
    sc_assert(h3.terminated() == false);
    sc_assert(h3.dynamic() == false);
    sc_assert(h3 == h3);
    sc_assert(h3 != h);

    sc_process_handle h4;
    h4 = h3;
    sc_assert(h4 == h3);
    sc_assert(h4 != h);
  }

  void stat_method()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h3 = sc_get_current_process_handle();
    sc_assert(h3.valid() == true);
    sc_assert(strcmp(h3.name(), "top.m.stat_method") == 0);
    sc_assert(h3.proc_kind() == SC_METHOD_PROC_);
    sc_assert(h3.get_process_object() != 0);
    std::vector<sc_object*> children3 = h3.get_child_objects();
    sc_assert(children3.size() == 0);
    sc_assert(h3.get_parent_object() != 0);
    sc_assert(h3.terminated() == false);
    sc_assert(h3.dynamic() == false);
    sc_assert(h3 == h3);
    sc_assert(h3 != h);

    sc_process_handle h4;
    h4 = h3;
    sc_assert(h4 == h3);
    sc_assert(h4 != h);
  }

  void dyn_thread()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h3 = sc_get_current_process_handle();
    sc_assert(h3.valid() == true);
    sc_assert(strcmp(h3.name(), "top.m.T.dyn_thread") == 0);
    sc_assert(h3.proc_kind() == SC_THREAD_PROC_);
    sc_assert(h3.get_process_object() != 0);
    std::vector<sc_object*> children3 = h3.get_child_objects();
    sc_assert(children3.size() == 0);
    sc_assert(h3.get_parent_object() != 0);
    sc_assert(h3.terminated() == false);
    sc_assert(h3.dynamic() == true);
    sc_assert(h3 == h3);
    sc_assert(h3 != h);

    sc_process_handle h4(h3);
    sc_assert(h4 == h3);
  }

  void dyn_method()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h3 = sc_get_current_process_handle();
    sc_assert(h3.valid() == true);
    sc_assert(strcmp(h3.name(), "top.m.T.dyn_method") == 0);
    sc_assert(h3.proc_kind() == SC_METHOD_PROC_);
    sc_assert(h3.get_process_object() != 0);
    std::vector<sc_object*> children3 = h3.get_child_objects();
    sc_assert(children3.size() == 0);
    sc_assert(h3.get_parent_object() != 0);
    sc_assert(h3.terminated() == false);
    sc_assert(h3.dynamic() == true);
    sc_assert(h3 == h3);
    sc_assert(h3 != h);

    sc_process_handle h4(h3);
    sc_assert(h4 == h3);
  }

  void CT()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h2 = sc_get_current_process_handle();
    sc_assert(h2.valid() == true);
    sc_assert(strcmp(h2.name(), "top.m.CT") == 0);
    sc_assert(h2.proc_kind() == SC_CTHREAD_PROC_);
    sc_assert(h2.get_process_object() != 0);
    std::vector<sc_object*> children2 = h2.get_child_objects();
    sc_assert(children2.size() == 0);
    sc_assert(h2.get_parent_object() == this);
    sc_assert(h2.terminated() == false);
    sc_assert(h2.dynamic() == false);
    sc_assert(h2 == h2);
    sc_assert(h2 != h);
  }

  void ME()
  {
    sc_process_handle h;
    invalid_handle_check(h);

    sc_process_handle h2 = sc_get_current_process_handle();
    sc_assert(h2.valid() == true);
    sc_assert(strcmp(h2.name(), "top.m.ME") == 0);
    sc_assert(h2.proc_kind() == SC_METHOD_PROC_);
    sc_assert(h2.get_process_object() != 0);
    std::vector<sc_object*> children2 = h2.get_child_objects();
    sc_assert(children2.size() == 0);
    sc_assert(h2.get_parent_object() == this);
    sc_assert(h2.terminated() == false);
    sc_assert(h2.dynamic() == false);
    sc_assert(h2 == h2);
    sc_assert(h2 != h);
  }

};

struct Top: sc_module
{
  M *m;
  sc_clock clk;
  Top(sc_module_name)
  {
    m = new M("m");
    m->clk(clk);
  }
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent except for warning messages" << endl;

  sc_process_handle h;
  invalid_handle_check(h);

  Top top("top");
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
