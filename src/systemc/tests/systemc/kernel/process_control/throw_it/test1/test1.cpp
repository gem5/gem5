//----------------------------------------------------------------------
//   Copyright 2009 Cadence Design Systems, Inc.
//   All Rights Reserved Worldwide
//----------------------------------------------------------------------

#include <systemc.h>

class my_exception {
public:
  my_exception(const char* s) : s_(s) { }
  const char* message() { return s_.c_str(); }
protected:
  std::string s_;
};

SC_MODULE(top) {
public:
  SC_CTOR(top) {
    SC_THREAD(victim);
    h = sc_get_current_process_handle();
    SC_THREAD(perpetrator);
  }

  void victim() {
    try {
      cerr << sc_time_stamp() << ": starting victim thread" << endl;
      ::sc_core::wait(100, SC_NS);
    }
    catch (my_exception& x) {
      cerr << sc_time_stamp() << ": in victim thread, caught exception "
           << x.message() << ", exiting" << endl;
      return;
    }
  }

  void perpetrator() {
    wait(10, SC_NS);
    cerr << sc_time_stamp() 
         << ": in perpetrator throwing exception in victim " 
         << endl;
    h.throw_it(my_exception("from pepetrator"));
    cerr << sc_time_stamp() 
         << ": in perpetrator after throwing exception in victim " 
         << endl;
  }

protected:
  sc_process_handle h;
};

int sc_main (int argc, char *argv[])
{
  top t("top");
  sc_start();
  return 0;
}

