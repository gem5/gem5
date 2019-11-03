
/*****************************************************************************

  The following code is derived, directly or indirectly, from the SystemC
  source code Copyright (c) 1996-2014 by all Contributors.
  All Rights reserved.

  The contents of this file are subject to the restrictions and limitations
  set forth in the SystemC Open Source License (the "License");
  You may not use this file except in compliance with such restrictions and
  limitations. You may obtain instructions on how to receive a copy of the
  License at http://www.accellera.org/. Software distributed by Contributors
  under the License is distributed on an "AS IS" basis, WITHOUT WARRANTY OF
  ANY KIND, either express or implied. See the License for the specific
  language governing rights and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_method_reset_throw.cpp -- 

  Original Author: Bishnupriya Bhattacharya, Cadence Design Systems, 2012-08-07

 *****************************************************************************/

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>

class my_exception
{
public:
  explicit my_exception(const char* s) : s_(s) { }
  const char* message() const { return s_.c_str(); }
protected:
  std::string s_;
};

SC_MODULE(sctop)
{
public:
   SC_CTOR(sctop)
   {
        SC_THREAD(run);
        SC_METHOD(m1); dont_initialize();
        method_handle = sc_get_current_process_handle();
        SC_THREAD(throwee1);
        throwee1_h = sc_get_current_process_handle();
   }

   void run() {
      wait (5, SC_NS);
      cout <<  sc_time_stamp() << ": reset method m1" << endl;
      method_handle.reset();
      cout <<  sc_time_stamp() << ": after reset of method m1" << endl;
   }

   void m1()
   {
      cout << sc_time_stamp() << ": in m1" << endl;
      cout << sc_time_stamp() << ": in m1() "
           << "throwing exception in throwee1" << endl;

      throwee1_h.throw_it(
         my_exception("thrown in throwee1 from m1()")
      );

      cout << sc_time_stamp() << ": in m1() "
           << "after throwing exception in throwee1" << endl;
  }

  void throwee1()
  {
    // catch exception and exit
    while (1) {
       try {
         wait(50, SC_NS);
         cerr << sc_time_stamp() << ": in throwee1, normal flow" << endl;
       }
       catch (my_exception const & x) {
         cerr << sc_time_stamp() << ": in throwee1, caught exception "
              << endl;
         return;
       }
    }
  }

protected:
  sc_process_handle method_handle;
  sc_process_handle throwee1_h;
};


int sc_main (int, char*[])
{
  sc_report_handler::set_actions( "disable() or dont_initialize() "
          "called on process with no static sensitivity, it will be orphaned",
          SC_DO_NOTHING );
  sctop top1("Top1");
  sc_start(10, SC_NS);
  return 0;
} 
