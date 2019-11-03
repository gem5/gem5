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

  new_prop2.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

static int (*print_func)(const char*, ...);

struct new_prop : sc_module {
    sc_in<bool> i[16];

    void my_print(const char*);

    void async0() { my_print("async0"); }
    void async1() { my_print("async1"); }
    void async2() { my_print("async2"); }
    void async3() { my_print("async3"); }

    void aproc0() { wait(); while (1) { my_print("aproc0"); wait(); } }
    void aproc1() { wait(); while (1) { my_print("aproc1"); wait(); } }
    void aproc2() { wait(); while (1) { my_print("aproc2"); wait(); } }
    void aproc3() { wait(); while (1) { my_print("aproc3"); wait(); } }

    SC_CTOR(new_prop) {
        SC_METHOD(async0);
        sensitive << i[4].neg();
        sensitive << i[12].neg();
        sensitive << i[13].neg();
        sensitive << i[14].neg();
        sensitive << i[15].neg();

        SC_METHOD(async1);
        sensitive << i[1].pos();
        sensitive << i[5].pos();
        sensitive << i[7].pos();
        sensitive << i[9].pos();

        SC_METHOD(async2);
        sensitive << i[5].neg();
        sensitive << i[6].neg();
        sensitive << i[13].pos();
        sensitive << i[15].pos();
        
        SC_METHOD(async3);
        sensitive << i[3].pos();
        sensitive << i[5].neg();
        sensitive << i[7].neg();
        sensitive << i[11].pos();

        SC_THREAD(aproc0);
        sensitive << i[2].pos();

        SC_THREAD(aproc1);
        sensitive << i[3].pos();
        sensitive << i[12].neg();
        sensitive << i[13].neg();
        sensitive << i[14].neg();
        sensitive << i[15].neg();

        SC_THREAD(aproc2);
        sensitive << i[8].neg();
        sensitive << i[9].neg();
        sensitive << i[10].neg();
        sensitive << i[11].neg();

        SC_THREAD(aproc3);
        sensitive << i[6].pos();
        sensitive << i[7].pos();
        sensitive << i[10].pos();
        sensitive << i[11].pos();
        sensitive << i[14].pos();
        sensitive << i[15].pos();
    }
};

void
new_prop::my_print(const char* p)
{
    (*print_func)("%s executed on:\n", p);
    for (int j = 0; j < 16; ++j) {
        if (i[j].posedge()) {
            (*print_func)("\tposedge i[%d]\n", j);
        }
        if (i[j].negedge()) {
            (*print_func)("\tnegedge i[%d]\n", j);
        }
    }
}

static int
dont_print(const char* fmt, ...)
{
    return 0;
}

int
sc_main(int,char**)
{
    // sc_clock i[16];
    sc_signal<bool> i[16];

    new_prop np("np");

    for( int j = 0; j < 16; ++ j ) {
        np.i[j]( i[j] );
    }

    for (int j = 0; j < 16; ++j) {
        i[j] = 0;
    }

    print_func = &dont_print;
    sc_start(0, SC_NS);

    print_func = &printf;

    for (int k = 0; k < 16; ++k) {
        i[k] = ! i[k].read();
        sc_start(1, SC_NS);
        i[k] = ! i[k].read();
        sc_start(1, SC_NS);
    }

    fflush( stdout );

    return 0;
}
