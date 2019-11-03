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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
Hello,

sorry for asking again about the sc_start/sc_cycle problem but...

The following program causes trouble (SystemC V2.0b2):
*/

#include "systemc.h"

SC_MODULE(createpulse)
 {
  public:
  sc_in_clk i_clk;
  private:
  int trigger1,trigger2;
  void pulse()
   {
    while(true)
     {
      wait();
      cout << sc_time_stamp() << ": trigger1 : " <<trigger1++ << endl;
      wait(1,SC_NS);
      cout << sc_time_stamp() << ": trigger2 : " <<trigger2++ << endl;
     }
   }
  public:
  SC_CTOR(createpulse)
   {
    SC_THREAD(pulse);
    sensitive << i_clk.pos();
    trigger1 = 0;
    trigger2 = 0;
   }
 };

// createpulse dut("testpulse");

int sc_main(int argc, char *argv[])
 {
  int i;
  sc_trace_file *tf;
  sc_signal<bool> clk1;

  sc_set_time_resolution(1,SC_NS);
  sc_set_default_time_unit(1,SC_NS);

  // sc_clock dummy( "dummy", 2, SC_NS );

  createpulse dut("testpulse");

  dut.i_clk(clk1);      // see other posting

  tf=sc_create_vcd_trace_file("vcdtrace");
  sc_trace(tf,clk1,"clock");

  // sc_initialize();      // comment out for sc_start version
  for(i=0;i<10;i++)
   {
    clk1=0;
    // sc_cycle(5,SC_NS);  // change to sc_start
    sc_start( 5, SC_NS );
    clk1=1;
    // sc_cycle(5,SC_NS);  // change to sc_start
    sc_start( 5, SC_NS );
   }

  cout << "finishing at " << sc_time_stamp() << endl;
  sc_close_vcd_trace_file(tf);

  return(EXIT_SUCCESS);
 }

/*
With this programm, the clk1 is generated as can be seen in the trace file. 
But the pulse procedure gets stuck in the second wait function. With SystemC 
V1.x, this worked with replacing sc_cycle with sc_start (and removing the 
sc_initialize), however calling sc_start multiple was an undocumented feature 
and it doesnt work in V2.0b2 (no clk is generated).
I know that this problem can be solved by creating an own clock generation 
module using wait()s and just a single sc_start in sc_main, but IMHO 
sometimes it is desirable to do it the way shown above. So is this possible 
with SystemC V2.0 ?

Regards, Sven Heithecker

-- 
Sven Heithecker                            IDA, Hans-Sommer-Str. 66
Technical University of Braunschweig       38106 Braunschweig
Tel. +49-(0)531-391-3751(voice)/4587(fax)  Germany
http://www.ida.ing.tu-bs.de/~svenh         heithecker@ida.ing.tu-bs.de
*/
