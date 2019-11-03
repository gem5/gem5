/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  Original Author: Stuart Swan, Cadence Design Systems, Inc., 2002-10-22

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: test01.cpp,v $
// Revision 1.3  2011/02/08 06:49:37  acg
//  Andy Goodrich: removed carriage returns.
//
// Revision 1.2  2011/02/01 17:17:37  acg
//  Andy Goodrich: update of copyright notice, added visible CVS logging.
//

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc.h>


int test_function(double d)
{
  cout << endl << sc_time_stamp() << ": " << sc_get_current_process_handle().name()
       << ": Test_function sees " << d << endl;
  return int(d);
}

void void_function(double d)
{
  cout << endl << sc_time_stamp() << ": " << sc_get_current_process_handle().name()
       << ": void_function sees " << d << endl;
}

int ref_function(const double& d)
{
  cout << endl << sc_time_stamp() << ": " << sc_get_current_process_handle().name()
       << ": ref_function sees " << d << endl;
  return int(d);
}


class top : public sc_module
{
public:
  SC_HAS_PROCESS(top);

  top(sc_module_name name) : sc_module(name) 
  {
     SC_THREAD(main);
  }

  void main()
  {
    int r;
    sc_event e1, e2, e3, e4;

    cout << endl;

    e1.notify(100, SC_NS);

    // Spawn several threads that co-operatively execute in round robin order

    SC_FORK
      sc_spawn(&r,
        sc_bind(&top::round_robin, this, "1", sc_ref(e1), sc_ref(e2), 3), "t1") ,
      sc_spawn(&r,
        sc_bind(&top::round_robin, this, "2", sc_ref(e2), sc_ref(e3), 3), "t2") ,
      sc_spawn(&r,
        sc_bind(&top::round_robin, this, "3", sc_ref(e3), sc_ref(e4), 3), "t3") ,
      sc_spawn(&r,
        sc_bind(&top::round_robin, this, "4", sc_ref(e4), sc_ref(e1), 3), "t4") ,
    SC_JOIN

    cout << "Returned int is " << r << endl;
    cout << endl << endl;

    // Test that threads in thread pool are successfully reused ...

    for (int i = 0 ; i < 10; i++)
      sc_spawn(&r, sc_bind(&top::wait_and_end, this, i));

    wait(20, SC_NS);

    // Test thread reuse

    for (int i = 0 ; i < 10; i++)
      sc_spawn(&r, sc_bind(&top::wait_and_end, this, i));

    wait(20, SC_NS);

    // Demo of a function rather than method call, & use return value ...
    
    wait( sc_spawn(&r, sc_bind(&test_function, 3.14159)).terminated_event() );

    cout << "Returned int is " << r << endl;

    // demo sc_spawn_options usage

    sc_spawn_options ops;
    ops.set_stack_size(0);
    sc_process_handle handle1 = sc_spawn(
      sc_bind(&void_function, 1.2345), "void_function", &ops
    );
    wait(handle1.terminated_event());

    double d = 9.8765;
    wait( sc_spawn(&r, sc_bind(&ref_function, sc_cref(d))).terminated_event() );

    cout << "Returned int is " << r << endl;

    cout << endl << "Done." << endl;
  }

  int round_robin(const char *str, sc_event& receive, sc_event& send, int cnt)
  {
    while (--cnt >= 0)
    {
      wait(receive);
      cout << sc_time_stamp() << ": " << sc_get_current_process_handle().name()
           << ": Round robin thread " << str << endl;
      wait(10, SC_NS);
      send.notify();
    }

    return 0;
  }

  int wait_and_end(int i)
  {
    wait( i + 1, SC_NS);
    cout << sc_time_stamp() << ": " << sc_get_current_process_handle().name()
         << ": Thread " << i << " ending." << endl;
    return 0;
  }
};

int sc_main (int argc , char *argv[]) 
{
  top top1("Top1");
  sc_start();

  return 0;
}
