
#include <systemc.h>

// Tests for 2.0.1 extended coding styles, illustrating that SystemC is a class library, not a syntax

static int global_count = 0;

SC_MODULE(Mod)
{
  SC_CTOR(Mod)
  {
    sc_assert(std::string(this->kind()) == "sc_module");
  }
};



SC_MODULE(Mod0)
{
  SC_CTOR(Mod0)
  {
    sc_assert(std::string(basename()) == "mod0");
    SC_METHOD(method);
    dont_initialize();
    method();             //// Calling an SC_METHOD member function directly DOULOS009

    for (int i = 0; i < 3; i++)
    {
      SC_THREAD(thread);  //// Registering the same function multiple times DOULOS046
                          //// Get warnings re-defining sc_object names, multiple threads created
    }
    SC_THREAD(thread);
    SC_THREAD(thread);  //// Compile-time error - 'threadhandle' redefinition

    f();
  }
  void method(void) {
    sc_assert(sc_get_current_process_handle().proc_kind() == SC_METHOD_PROC_);
  ++ global_count;
  }
  void thread(void) {
  sc_assert(sc_get_current_process_handle().proc_kind() == SC_THREAD_PROC_);
  ++ global_count;
  }

  void f() { SC_THREAD(g); } //// Process registered in member function called from constructor DOULOS007
  void g()
  {
    sc_assert(std::string(sc_get_current_process_handle().name()) == "top.mod0.g");
  ++ global_count;
  }
};


struct Chan               //// Pseudo-channel used for port-less interprocess communication
                          //// but not derived from sc_module, sc_prim_channel or sc_interface DOULOS048
{
  void write(int i) { wait(10, SC_NS); data = i; e.notify(); }
  void read(int& i) { wait(e); i = data; }
  int data;
  bool wr, re;
  sc_event e;
};

struct MyMod: Mod         //// Class derived from an SC_MODULE DOULOS050
{
  sc_in<bool> p;

  MyMod(sc_module_name n)
  : Mod(n)
  {
    SC_THREAD(p1);
    SC_THREAD(p2);
  }

  Chan ch;                //// Instance of pseudo-channel DOULOS048
  void p1() { ch.write(333); }
  void p2() { int i; ch.read(i); sc_assert(i == 333); }

  SC_HAS_PROCESS(MyMod);
};



struct C0: sc_module, virtual sc_interface //// Combining channel and interface in one class DOULOS015
{
  void write(int i) { data = i; }
  void read(int& i) { i = data; }
  int data;

  SC_CTOR(C0)
  {
    SC_THREAD(action);
  }
  void action() {
  sc_assert(std::string(sc_get_current_process_handle().get_process_object()->basename()) == "action");
  }
};


struct C1: virtual public sc_interface //// Combining channel and interface in one class DOULOS015
{
  void write(int i) { data = i; }
  void read(int& i) { i = data; }
  int data;
};


struct I_F: virtual public sc_interface
{
  virtual void method() = 0;
};

struct C2: I_F            //// Channel derived from neither sc_module nor sc_prim_channel DOULOS049
{
  void method() {}
};

struct C3: sc_object, I_F //// Channel derived from sc_object only
                          //// (and derived from neither sc_module nor sc_prim_channel) DOULOS049
{
  void method() {}
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

struct Modtype: sc_module
{
  Modtype(sc_module_name n = sc_gen_unique_name("Modtype"))
  {
    if (global_count == 0)
      sc_assert(std::string(basename()) == "mt");
    else
    {
      std::string s = std::string(basename());
      sc_assert(s.substr(0,7) == "Modtype");
      sc_assert(s.size() > 7);
      check_form_of_suffix(s.substr(7));
    }
    ++ global_count;
  }
  void dump() {
    sc_assert(std::string(sc_get_current_process_handle().get_process_object()->basename()) == "thread");
    ++ global_count;
  }
};

typedef sc_signal<Modtype*> MS; //// Pointer-to-module used as the type of a signal DOULOS051



template<class T>
struct S
{
  T t;
};



SC_MODULE(Moda)
{
  sc_port<C0> p0;
  sc_port<C1> p1;
  sc_port<C2> p2;
  sc_port<C3> p3;
  sc_port<MS> p4;

  Modtype mt;

  SC_CTOR(Moda): mt("mt")
  {
  sc_assert(std::string(name()) == "top.moda");
    SC_THREAD(thread);
  }
  void thread();
};

SC_MODULE(Modb)
{
  sc_port<C0> p0;
  sc_port<C1> p1;
  sc_port<C2> p2;
  sc_port<C3> p3;
  sc_port<MS> p4;

  SC_CTOR(Modb)
  {
  sc_assert(std::string(name()) == "top.modb");
    SC_THREAD(thread);
    SC_THREAD(funny);
  }
  void thread();
  void funny();
};

void Moda::thread()
{
  p1->write(999);
  p2->method();
  p3->method();
  p4->write(&mt);
}
void Modb::thread()
{
  wait(SC_ZERO_TIME);
  int i; p1->read(i);
  sc_assert(i == 999);
  p2->method();
  p3->method();
  wait(SC_ZERO_TIME);
  (p4->read())->dump();
}

void Modb::funny()  { C1 c1; C2 c2; C3 c3; }




struct Link
{
  Link *link;
  sc_module      *m;       //// Pointer-to-module DOULOS006
  sc_signal<int> *s;       //// Pointer-to-signal
  sc_in<int>     *p;       //// Pointer-to-port DOULOS008
};

SC_MODULE(Top)
{

  SC_MODULE(Nested)        //// Nested modules DOULOS005
  {
    sc_in<int> *pp;        //// Pointer-to-port DOULOS008

    SC_CTOR(Nested)
    {
      SC_METHOD(action);

      pp = new sc_in<int>; //// Dynamic port instantiation DOULOS008

      sensitive << *pp;    //// Sensitivity separated from SC_METHOD DOULOS011
    }

  void action() { op = sc_min(3, (*pp).read() + 1); }

    sc_out<int> op;        //// Out-of-order declaration DOULOS052
  };


  Nested n;
  Link* link;
  sc_signal<bool> b;
  sc_signal<int> *sig;      //// Pointer-to-signal

  MyMod mymod;
  Moda moda;
  Modb modb;
  C0 c0;
  C1 c1;
  C2 c2;
  C3 c3;

  S<Modtype> Sm;            //// Using sc_module as a template parameter DOULOS051
  MS ms;                    //// Pointer-to-module used as the type of a signal DOULOS051

  Mod0 mod0;

  SC_CTOR(Top)
  : n("n"), mymod("mymod"), moda("moda"), modb("modb"), c0("c0"), mod0("mod0")
  {
    link = new Link;
    link->m = new Mod("mod_1");
    link->p = new sc_in<int>;
    link->link = new Link;
    link->link->m = new Mod("mod_2");  //// Buried dynamic module instantiation DOULOS002
    link->link->s = new sc_signal<int>;//// Buried dynamic channel instantiation
    link->link->p = new sc_in<int>;    //// Buried dynamic port instantiation DOULOS008

    sig = new sc_signal<int>;          //// Dynamic channel instantiation
    n.op(*sig);
    (*(n.pp)).bind(*sig);              //// Binding dynamically allocated port DOULOS008

    mymod.p(b);

    moda.p0(c0);
    moda.p1(c1);
    moda.p2(c2);
    moda.p3(c3);
    moda.p4(ms);

    modb.p0(c0);
    modb.p1(c1);
    modb.p2(c2);
    modb.p3(c3);
    modb.p4(ms);
  }

};


int sc_main(int argc, char* argv[])
{
  cout << "Should be silent except for some renaming warnings..." << endl;

  sc_signal<int> s;
  Top top("top");
  top.link->p->bind(s);
  top.link->link->p->bind(s);         //// Binding dynamically allocated port DOULOS008

  sc_start();
  sc_assert(global_count == 10);
  sc_assert(top.sig->read() == 3);

  cout << endl << "Success" << endl;
  return 0;
}
