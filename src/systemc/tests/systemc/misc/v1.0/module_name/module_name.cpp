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

  module_name.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int numbers[] = { 49597, 41218, 20635, 40894, 16767, 17233, 36246, 28171, 60879, 49566, 10971, 24107, 30561, 49648, 50031, 12559, 23787, 35674, 43320, 37558, 840, 18689, 62466, 6308, 46271, 49801, 43433, 22683, 35494, 35259, 29020, 19555, 10941, 49656, 60450, 27709, 1353, 31160, 55880, 62232, 15190, 1315, 20803, 45751, 50963, 5298, 58311, 9215, 2378 };

int numbers_index = 0;

struct example : sc_module {
    sc_in_clk  clk;
    sc_in<int> a;
    sc_in<int> b;
    sc_out<int> c;

    sc_signal<int> d;
    sc_signal<int> e;

    void block_a();
    void block_b();
    void block_c();
    void block_d();
    void block_e();
    void block_f();

    SC_CTOR(example)
    {
        SC_METHOD( block_a );
        sensitive << a;
        sensitive << b;

        SC_METHOD( block_b );
        sensitive << a << b;

        SC_METHOD( block_c );
        sensitive << d << e;

        SC_CTHREAD( block_d, clk.neg() );

        SC_CTHREAD( block_e, clk.pos() );

        SC_CTHREAD( block_f, clk.pos() );
    }
};

void
example::block_a()
{
    d = a + b;
}

void
example::block_b()
{
    e = a - b;
}

void
example::block_c()
{
    c = d * e;
}

void
example::block_d()
{
    int i = 0;
    while (true) {
        cout << "block_d " << i << endl;
        i++;
        wait();
    }
}

void
example::block_e()
{
    int i = 0;
    while (true) {
        cout << "block_e " << i << endl;
        i++;
        wait();
    }
}

void
example::block_f()
{
    int i = 32;
    while (true) {
        cout << "block_f " << i << endl;
        i++;
        wait();
    }
}

struct tb : sc_module {
    sc_in_clk   clk;
    sc_out<int> a;

    void tb_proc();

    SC_CTOR(tb)
    {
        SC_CTHREAD( tb_proc, clk.pos() );
    }
};

void
tb::tb_proc()
{
    while (true) {
        a = numbers[numbers_index % (sizeof(numbers)/sizeof(numbers[0]))];
        numbers_index++;
        cout << "tb_proc " << endl;
        wait();
    }
}

struct tb2 : sc_module {
    sc_in_clk   clk;
    sc_out<int> b;

    void tb2_proc();

    SC_CTOR(tb2)
    {
        SC_CTHREAD( tb2_proc, clk.pos() );
    }
};

void
tb2::tb2_proc()
{
    while (true) {
        b = numbers[numbers_index % (sizeof(numbers)/sizeof(numbers[0]))];
        numbers_index++;
        cout << "tb2_proc " << endl;
        wait();
    }
}

SC_MODULE( monitor )
{
    SC_HAS_PROCESS( monitor );

    const sc_signal<int>& a;
    const sc_signal<int>& b;
    const sc_signal<int>& c;

    monitor( sc_module_name,
             const sc_signal<int>& A,
             const sc_signal<int>& B,
             const sc_signal<int>& C ) :
        a(A), b(B), c(C)
    {
	SC_METHOD( entry );
        sensitive << a;
        sensitive << b;
        sensitive << c;
    }
    void entry();
};

void
monitor::entry()
{
    if (a.event()) cout << "a = " << a << endl;
    if (b.event()) cout << "b = " << b << endl;
    if (c.event()) cout << "c = " << c << endl;
}

int
sc_main( int argc, char* argv[] )
{
    sc_signal<int> a("a");
    sc_signal<int> b("b");
    sc_signal<int> c("c");
    sc_clock clk("clk", 10, SC_NS);

    example ex1("ex1");
    ex1(clk, a, b, c);

    tb tbb1("tbb1");
    tbb1(clk, a);

    tb2 tbb2("tbb2");
    tbb2(clk, b);

    monitor mon("mon", a, b, c);

    sc_start(200, SC_NS);
    return 0;
}
