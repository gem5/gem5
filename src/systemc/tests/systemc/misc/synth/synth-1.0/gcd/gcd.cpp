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

  gcd.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

struct gcd_cc : public sc_module {
    sc_in_clk        clk;
    sc_in<bool>      reset;
    sc_in<unsigned>  a;
    sc_in<unsigned>  b;
    sc_out<unsigned> c;
    sc_out<bool>     ready;

    void gcd_compute();

    SC_HAS_PROCESS( gcd_cc );

    gcd_cc( sc_module_name name )
    {
        SC_CTHREAD( gcd_compute, clk.pos() );
        reset_signal_is(reset,true);
    }
};

void
gcd_cc::gcd_compute()
{
    unsigned tmp_a = 0;
    wait();         // Note that this wait() is required, otherwise,
                    // the reset is wrong!  This is a problem with BC,
                    // not our frontend.

    while (true) {
        unsigned tmp_b;

        c = tmp_a;
        ready = true;
        wait();

        tmp_a = a;
        tmp_b = b;
        ready = false;
        wait();

        while (tmp_b != 0) {

            unsigned tmp_c = tmp_a;
            tmp_a = tmp_b;
            wait();

            while (tmp_c >= tmp_b) {
                tmp_c = tmp_c - tmp_b;
                wait();
            }

            tmp_b = tmp_c;
            wait();
        }
    }
}

static int numbers[] = { 49597, 41218, 20635, 40894, 16767, 17233, 36246, 28171, 60879, 49566, 10971, 24107, 30561, 49648, 50031, 12559, 23787, 35674, 43320, 37558, 840, 18689, 62466, 6308, 46271, 49801, 43433, 22683, 35494, 35259, 29020, 19555, 10941, 49656, 60450, 27709, 1353, 31160, 55880, 62232, 15190, 1315, 20803, 45751, 50963, 5298, 58311, 9215, 2378 };
static unsigned numbers_index = 0;

struct testbench : public sc_module {
    sc_in_clk          clk;
    sc_inout<bool>     reset;
    sc_in<bool>        ready;
    sc_inout<unsigned> a;
    sc_inout<unsigned> b;
    sc_in<unsigned>    c;

    void reset_gen();
    void stimu_gen();
    void display();

    SC_HAS_PROCESS( testbench );

    testbench( sc_module_name name )
    {
        SC_CTHREAD( reset_gen, clk.pos() );
        SC_CTHREAD( stimu_gen, clk.pos() );
        SC_METHOD( display );
        sensitive << ready;
    }
};

void
testbench::reset_gen()
{
    reset = 0;
    wait();
    reset = 1;
    wait();
    wait();
    reset = 0;
    wait();
    /* die */
}

void
testbench::stimu_gen()
{
    while (true) {
        do { wait(); } while (ready == 0);
        a = (unsigned) numbers[numbers_index++ % (sizeof(numbers)/sizeof(numbers[0]))];
        b = (unsigned) numbers[(numbers_index*numbers_index) % (sizeof(numbers)/sizeof(numbers[0]))];
        numbers_index++;
    }
}

void
testbench::display()
{
    if (ready) {
        cout << "reset = " << reset << " ready = " << ready
             << " a = " << a << " b = " << b << " c = " << c << endl;
    }
}

int sc_main(int argc, char* argv[] )
{
    sc_signal<unsigned> a("a"), b("b"), c("c");
    sc_clock            clk("clk", 20, SC_NS);
    sc_signal<bool>     reset("reset"), ready("ready");

    a = 0;
    b = 0;
    c = 0;
    reset = false;
    ready = false;

    gcd_cc gcd("gcd");
    gcd(clk, reset, a, b, c, ready);

    testbench tb("tb");
    tb(clk, reset, ready, a, b, c);

    sc_start(2000000, SC_NS);
    return 0;
}
