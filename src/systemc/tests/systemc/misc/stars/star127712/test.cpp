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
If a process is sensitive to sc_port<some_if<T>, N> and
this port is bind to two signals, the process is responding
to only the events on first signal, it's not responding to
events on second (or other) signals. it seems to me as a
bug. how to resolve this problem?? 

Thanks in advance,
kiran

when i run the following program i got this output.
0: Method Activated
0: Method Activated
3: Method Activated

but expected output is,
0: Method Activated
0: Method Activated
1: Method Activated
2: Method Activated
3: Method Activated
*/

#include "systemc.h"

SC_MODULE(tst) {
  sc_port<sc_signal_in_if<bool>, 2> INP;

  void print_mthd() {
    cout <<sc_time_stamp()<<": Method Activated"<<endl;
  }
 
  SC_CTOR(tst) {
    SC_METHOD(print_mthd);
    sensitive << INP;
  }
};

int sc_main(int argc, char* argv[]) {

  sc_signal<bool> INP1, INP2;
  
  tst tsti("tsti");
  tsti.INP(INP1);
  tsti.INP(INP2);

  sc_start(0, SC_NS);

  INP1.write(1);
  sc_start(1, SC_NS);
  INP2.write(1);
  sc_start(1, SC_NS);
  INP2.write(0);
  sc_start(1, SC_NS);
  INP1.write(0);
  sc_start(1, SC_NS);

  return 0;
}


/*
=====
M.N.V.Satya Kiran,M.Tech
Project Trainee, ED&T Synthesis,
Building: WAY- 3.24,
Phone: +31-40-2743924.
NatLab-Philips Research Laboratories,
Prof.Holstlaan 4, 5656 AA Eindhoven,
The Netherlands.
*/
