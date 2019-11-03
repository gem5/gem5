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

  wait.cpp -- 

  Original Author: Daniel Aarno, Intel, Corp 2015-07-23

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc>

using sc_core::sc_time;
using sc_core::SC_US;

namespace {
int first_line = 0;

bool endsWith(const std::string &str, const std::string &ending) {
    if (ending.size() > str.size()) {
        return false;
    }
    return std::equal(ending.rbegin(), ending.rend(), str.rbegin());
}

bool waitFunIsTrue() {
  if (sc_core::sc_time_stamp() < sc_time(2000, SC_US)) {
    return false;
  }

  if (sc_core::sc_time_stamp() < sc_time(3000, SC_US)) {
    sc_time delay(10, SC_US);
    SC_WAITN(delay);
    return false;
  }

  if (sc_core::sc_time_stamp() < sc_time(4000, SC_US)) {
    return false;
  }

  return true;
}

SC_MODULE(Wait) {
  SC_CTOR(Wait) : clk( "clk", 10, SC_US, 0.5, 100, SC_US) {
    SC_THREAD(thread);
    sensitive << clk;
  }

  void thread() {
    sc_time delay(10, SC_US);
    wait(delay);  // 1st

    first_line = __LINE__ + 1;
    SC_WAITN(delay);  // Wait some time
    wait(delay);  // 2nd

    SC_WAIT();  // Wait for clk
    wait(delay);  // 3rd

    SC_WAIT_UNTIL(sc_core::sc_time_stamp() > sc_time(1000, SC_US));
    wait(delay);  // 4th

    SC_WAIT_UNTIL(waitFunIsTrue());
  }

  sc_core::sc_clock clk;
};

}  // namespace

int sc_main(int argc, char ** argv) {
  Wait w("dut");

  sc_core::sc_process_handle hnd( sc_core::sc_find_object("dut.thread") );
  const sc_core::sc_process_b *thread
    = dynamic_cast<sc_core::sc_process_b*>(hnd.get_process_object());
  sc_assert(hnd.valid() && thread);

  sc_assert(thread->file == NULL);  // 1st wait(delay)
  sc_assert(thread->lineno == 0);
  sc_core::sc_start(sc_time(15, SC_US));

  int lineno = first_line;
  sc_assert(endsWith(thread->file, "wait.cpp"));  // SC_WAITN
  sc_assert(thread->lineno == lineno);
  sc_core::sc_start(sc_time(10, SC_US));
  sc_assert(thread->file == NULL);  // 2nd wait(delay)
  sc_assert(thread->lineno == 0);
  sc_core::sc_start(sc_time(10, SC_US));

  lineno += 3;
  sc_assert(endsWith(thread->file, "wait.cpp"));  // SC_WAIT
  sc_assert(thread->lineno == lineno);
  sc_core::sc_start(sc_time(70, SC_US));
  sc_assert(thread->file == NULL);  // 3rd wait(delay)
  sc_assert(thread->lineno == 0);
  sc_core::sc_start(sc_time(10, SC_US));

  lineno += 3;
  sc_assert(endsWith(thread->file, "wait.cpp"));  // SC_WAIT_UNTIL
  sc_assert(thread->lineno == lineno);
  sc_core::sc_start(sc_time(900, SC_US));
  sc_assert(thread->file == NULL);  // 4th wait(delay)
  sc_assert(thread->lineno == 0);

  // Ensure that SC_WAIT_UNTIL can handle nested wait calls
  sc_core::sc_start(sc_time(10, SC_US));
  lineno += 3;
  sc_assert(endsWith(thread->file, "wait.cpp"));  // 2nd SC_WAIT_UNTIL
  sc_assert(thread->lineno == lineno);
  sc_core::sc_start(sc_time(980, SC_US));  // time should be 2005
  sc_assert(endsWith(thread->file, "wait.cpp"));  // SC_WAITN in waitFunIsTrue
  sc_assert(thread->lineno == 60);
  sc_core::sc_start(sc_time(95, SC_US));
  sc_assert(endsWith(thread->file, "wait.cpp"));
  sc_assert(thread->lineno == 60);
  sc_core::sc_start(sc_time(910, SC_US));
  sc_assert(endsWith(thread->file, "wait.cpp"));  // 2nd SC_WAIT_UNTIL
  sc_assert(thread->lineno == lineno);
  sc_core::sc_start(sc_time(1000, SC_US));
  sc_assert(thread->file == NULL);  // done
  sc_assert(thread->lineno == 0);

  return 0;
}
