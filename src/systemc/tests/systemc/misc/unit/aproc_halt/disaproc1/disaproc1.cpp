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

  disaproc1.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

int val1[17] = { 34329, 32492,  1034, 12000,  102, 12981,  1902, 19409,
                 10029,  2149, 12030, 20099,   90, 10009,  9345, 57483,
                 10903 };

int val2[19] = {   239,   923,  1240,   129,  191,   101,  1010,   190,
                 19820,  2349, 24039, 34728, 5745, 78234, 17838, 37482,
                 17498,  1347,  3721 };

SC_MODULE( aproc1 )
{
    SC_HAS_PROCESS( aproc1 );

    const sc_signal<bool>& a;
    const sc_signal<bool>& b;
          sc_signal<bool>& c;

    aproc1( sc_module_name NAME,

            const sc_signal<bool>& A,
            const sc_signal<bool>& B,
                  sc_signal<bool>& C )
        : a(A), b(B), c(C)
    {
        SC_THREAD( entry );
        sensitive << a << b;
    }
    void entry();
};

void
aproc1::entry()
{
    wait();
    c = a + b;
    cout << "c is (a + b)" << endl;
    wait();
    c = a - b;
    cout << "c is (a - b)" << endl;
    wait();
    cout << name() << " is exiting." << endl;
}


SC_MODULE( aproc2 )
{
    SC_HAS_PROCESS( aproc2 );

    const sc_signal<bool>& a;
    const sc_signal<bool>& b;
          sc_signal<bool>& d;

    aproc2( sc_module_name NAME,

            const sc_signal<bool>& A,
            const sc_signal<bool>& B,
                  sc_signal<bool>& D )
        : a(A), b(B), d(D)
    {
        SC_THREAD( entry );
        sensitive << a << b;
    }
    void entry();
};

void
aproc2::entry()
{
    wait();
    int loops = 0;
    while (true) {
        d = a * b;
        cout << "d is (a * b)" << endl;
        wait();
        if ((bool) b == 0) {
            d = a / (b + 1);
            cout << "d is (a / (b + 1))" << endl;
        } else {
            d = a / b;
            cout << "d is (a / b)" << endl;
        }
        wait();
        if (loops < 1) {
            // sc_assert( a.sensitive_aprocs_neg.size() == 2 );
            // sc_assert( a.sensitive_aprocs.size() == 2 );
        }
        if (loops > 5) {
            /* By this time aproc1 should have died. */
            // sc_assert( a.sensitive_aprocs_neg.size() == 1 );
            // sc_assert( a.sensitive_aprocs.size() == 1 );
        }
        loops++;
    }
}

SC_MODULE( sync1 )
{
    SC_HAS_PROCESS( sync1 );

    sc_in_clk clk;

          sc_signal<bool>& a;
          sc_signal<bool>& b;
    const sc_signal<bool>& c;
    const sc_signal<bool>& d;

    int count;
    sync1( sc_module_name NAME,
           sc_clock& CLK,
           sc_signal<bool>& A,
           sc_signal<bool>& B,
           const sc_signal<bool>& C,
           const sc_signal<bool>& D )
        : 
          a(A), b(B), c(C), d(D)

    {
        clk(CLK);
		SC_CTHREAD( entry, clk.pos() );
        count = 0;
    }
    void entry();
};

void
sync1::entry()
{
    while (true) {
        a = (val1[count % (sizeof(val1)/sizeof(val1[0]))] & 1);
        b = (val2[count % (sizeof(val2)/sizeof(val2[0]))] & 1);
        count++;
        wait();
        cout << "  a =  " << a;
        cout << "  b =  " << b;
        cout << "  c =  " << c;
        cout << "  d =  " << d << endl;
    }
}



int
sc_main(int argc, char** argv)
{
    sc_clock clk("clk");
    sc_signal<bool> a("a");
    sc_signal<bool> b("b");
    sc_signal<bool> c("c");
    sc_signal<bool> d("d");

    a = 0;
    b = 0;
    c = 0;
    d = 0;

    aproc1 p1("p1", a, b, c);
    aproc2 p2("p2", a, b, d);
    sync1  s1("s1", clk, a, b, c, d);

    sc_start(2000, SC_NS);
    return 0;
}
