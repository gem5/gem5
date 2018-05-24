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

  pr-207_mem.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

struct my_pair {
    char x;
    char y;
};

typedef sc_signal<bool>          sig_bool;
typedef sc_signal<char>          sig_char;
typedef sc_signal<unsigned char> sig_uchar;

SC_MODULE( pr207 )
{
    SC_HAS_PROCESS( pr207 );

    sc_in_clk clk;

    pr207(sc_module_name NAME,
          sc_clock& CLK,

          const sig_bool& RESET,
          const sig_bool& START,
          const sig_char& C1,
          const sig_char& C2,
          const sig_uchar& IDX1,
          const sig_uchar& IDX2,
                sig_char& D1,
                sig_char& D2,
                sig_bool& READY
          )
        : 
          reset(RESET),
          start(START),
          c1(C1),
          c2(C2),
          idx1(IDX1),
          idx2(IDX2),
          d1(D1),
          d2(D2),
          ready(READY)
    {
        clk(CLK);
		SC_CTHREAD( entry, clk.pos() );
        reset_signal_is(reset,true);
    }
    void entry();

    const sig_bool& reset;
    const sig_bool& start;
    const sig_char& c1;
    const sig_char& c2;
    const sig_uchar& idx1;
    const sig_uchar& idx2;
          sig_char& d1;
          sig_char& d2;
          sig_bool& ready;

};

void
pr207::entry()
{
    my_pair pair_array[10];

    ready = true;
    d1 = 0;
    d2 = 0;
    wait();
    while (true) {
        do { wait(); } while (start == 0);
        ready = false;
        wait();
        pair_array[idx1.read()].x = c1;
        pair_array[idx1.read()].y = c2;
        wait();
        pair_array[idx2.read()] = pair_array[idx1.read()];
        wait();
        char d1_tmp = pair_array[idx2.read()].y;
        wait();
        char d2_tmp = pair_array[idx2.read()].x;
        wait();
        d1 = d1_tmp;
        d2 = d2_tmp;
        ready = true;
    }
}

int sc_main(int argc, char* argv[] )
{
   return 0;
}

