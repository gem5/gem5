#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <systemc>
#include <cstring>
using namespace sc_core;
using namespace sc_dt;
using sc_core::wait;
using std::cout;
using std::endl;

// 8) get_parent_object, get_child_objects

struct Object: sc_object
{
  Object(const char* _n) : sc_object(_n) {}
};

SC_MODULE(M)
{
  sc_signal<bool> sig;
  sc_object* T_obj;

  SC_CTOR(M)
  {
    std::vector<sc_object*> children = sig.get_child_objects();
    sc_assert (children.size() == 0);
    SC_THREAD(T);
    sc_process_handle h = sc_get_current_process_handle();
    sc_assert (h.valid());
    T_obj = h.get_process_object();
    children = h.get_child_objects();
    sc_assert (children.size() == 0);
    children = h.get_process_object()->get_child_objects();
    sc_assert (children.size() == 0);

    Object obj("obj");
    children = obj.get_child_objects();
    sc_assert (children.size() == 0);
    sc_assert (obj.get_parent_object() == this);
  }

  void T()
  {
    sc_process_handle h = sc_get_current_process_handle();
    sc_assert (h.valid());
    sc_assert (h.dynamic() == false);
    sc_assert (h.get_process_object() == T_obj);
    sc_assert (h.get_parent_object() == this);
    std::vector<sc_object*> children = h.get_child_objects();
    sc_assert (children.size() == 0);
    children = h.get_process_object()->get_child_objects();
    sc_assert (children.size() == 0);

    Object obj("obj");
    children = obj.get_child_objects();
    sc_assert (children.size() == 0);
    sc_assert (obj.get_parent_object() == h.get_process_object());

    children = h.get_child_objects();
    sc_assert (children.size() == 1);
    sc_assert (children[0] == &obj);
    sc_assert (strcmp(children[0]->name(), "top.m.T.obj") == 0);
    sc_assert (children[0]->get_parent_object() == h.get_process_object());

    Object obj2("obj2");
    children = h.get_child_objects();
    sc_assert (children.size() == 2);
    sc_assert (children[0] == &obj);
    sc_assert (children[1] == &obj2);
    sc_assert (children[0]->get_parent_object() == h.get_process_object());
    sc_assert (children[1]->get_parent_object() == h.get_process_object());

    wait (1, SC_NS);
    sc_spawn(sc_bind(&M::dynamic_proc, this), "dynamic_proc");
    wait (1, SC_NS);

    sc_assert (h.valid());
  }

  sc_object* dynamic_proc_obj;

  void dynamic_proc()
  {
    sc_process_handle h = sc_get_current_process_handle();
    sc_assert (h.valid());
    dynamic_proc_obj = h.get_process_object();
    sc_assert (h.dynamic() == true);
    sc_assert (h.get_parent_object() == T_obj);
    std::vector<sc_object*> children = h.get_child_objects();
    sc_assert (children.size() == 0);
    children = h.get_process_object()->get_child_objects();
    sc_assert (children.size() == 0);

    Object obj("obj");
    children = obj.get_child_objects();
    sc_assert (children.size() == 0);
    sc_assert (obj.get_parent_object() == dynamic_proc_obj);

    children = h.get_child_objects();
    sc_assert (children.size() == 1);
    sc_assert (children[0] == &obj);
    sc_assert (strcmp(children[0]->name(), "top.m.T.dynamic_proc.obj") == 0);
    sc_assert (children[0]->get_parent_object() == dynamic_proc_obj);

    Object obj2("obj2");
    children = h.get_child_objects();
    sc_assert (children.size() == 2);
    sc_assert (children[0] == &obj);
    sc_assert (children[1] == &obj2);
    sc_assert (children[0]->get_parent_object() == dynamic_proc_obj);
    sc_assert (children[1]->get_parent_object() == dynamic_proc_obj);

    wait (1, SC_NS);
    sc_process_handle h2 = sc_spawn(sc_bind(&M::dynamic_proc2, this), "dynamic_proc2");
    wait (1, SC_NS);

    children = h.get_child_objects();
    sc_assert (children.size() == 3);
    sc_assert (children[0] == &obj);
    sc_assert (children[1] == &obj2);
    sc_assert (children[2] == h2.get_process_object());
    sc_assert (children[0]->get_parent_object() == dynamic_proc_obj);
    sc_assert (children[1]->get_parent_object() == dynamic_proc_obj);
    sc_assert (children[2]->get_parent_object() == dynamic_proc_obj);

    sc_assert (h.valid());
  }

  void dynamic_proc2()
  {
    sc_process_handle h = sc_get_current_process_handle();
    sc_assert (h.valid());
    sc_assert (h.dynamic() == true);
    sc_assert (h.get_parent_object() == dynamic_proc_obj);
    std::vector<sc_object*> children = h.get_child_objects();
    sc_assert (children.size() == 0);
    children = h.get_process_object()->get_child_objects();
    sc_assert (children.size() == 0);

    Object obj("obj");
    children = obj.get_child_objects();
    sc_assert (children.size() == 0);
    sc_assert (obj.get_parent_object() == h.get_process_object());

    children = h.get_child_objects();
    sc_assert (children.size() == 1);
    sc_assert (children[0] == &obj);
    sc_assert (strcmp(children[0]->name(), "top.m.T.dynamic_proc.dynamic_proc2.obj") == 0);
    sc_assert (children[0]->get_parent_object() == h.get_process_object());

    Object obj2("obj2");
    children = h.get_child_objects();
    sc_assert (children.size() == 2);
    sc_assert (children[0] == &obj);
    sc_assert (children[1] == &obj2);
    sc_assert (children[0]->get_parent_object() == h.get_process_object());
    sc_assert (children[1]->get_parent_object() == h.get_process_object());

    wait(10, SC_NS);

    sc_assert (h.valid());
  }
};

void g(sc_object* obj)
{
  sc_process_handle h = sc_get_current_process_handle();
  sc_object* this_process = h.get_process_object();
  sc_assert(this_process->get_parent_object() == obj);
}
void f()
{
  sc_object* obj = sc_get_current_process_handle().get_process_object();
  sc_process_handle h = sc_spawn(sc_bind(&g, obj));
  std::vector<sc_object*> children = obj->get_child_objects();
  sc_assert (children.size() == 1);
  wait(1, SC_NS);
}

SC_MODULE(Top)
{
  sc_in_clk clk;
  sc_port<sc_signal_in_if<int>, 1, SC_ZERO_OR_MORE_BOUND> p;
  sc_export<sc_signal_in_if<int> > xp;
  sc_signal<int> sig;

  M *m;
  SC_CTOR(Top)
    : clk("clk"), p("p"), xp("xp"), sig("sig")
  {
    m = new M("m");
    xp.bind(sig);

    SC_THREAD(p1);
    SC_CTHREAD(p2,clk);
    SC_METHOD(p3);
    sc_spawn(&f,"p4");

    sc_object* parent = this->get_parent_object();
    sc_assert (parent == 0);

    std::vector<sc_object*> children = this->get_child_objects();
    sc_assert (children.size() == 9);
    sc_assert (strcmp(children[0]->name(), "top.clk") == 0);
    sc_assert (strcmp(children[1]->name(), "top.p") == 0);
    sc_assert (strcmp(children[2]->name(), "top.xp") == 0);
    sc_assert (strcmp(children[3]->name(), "top.sig") == 0);
    sc_assert (strcmp(children[4]->name(), "top.m") == 0);
    sc_assert (strcmp(children[5]->name(), "top.p1") == 0);
    sc_assert (strcmp(children[6]->name(), "top.p2") == 0);
    sc_assert (strcmp(children[7]->name(), "top.p3") == 0);
    sc_assert (strcmp(children[8]->name(), "top.p4") == 0);

    for (unsigned i = 0; i < children.size(); i++)
      sc_assert (children[i]->get_parent_object() == this);
  }
  void p1() {}
  void p2() {}
  void p3() {}
};

int sc_main(int argc, char* argv[])
{
  cout << "Should be silent..." << endl;

  Object obj("obj");
  std::vector<sc_object*> children = obj.get_child_objects();
  sc_assert (children.size() == 0);
  sc_assert (obj.get_parent_object() == 0);

  sc_signal<bool> clk;
  Top top("top");
  top.clk(clk);
  sc_start();

  cout << endl << "Success" << endl;
  return 0;
}
