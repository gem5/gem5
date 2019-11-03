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

  test3.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
  Corner case testing for new scheduler.
  Case 3: Checking sensitive_pos() and sensitive_neg() methods
*/

#include "systemc.h"

SC_MODULE( asyncproc )
{
  SC_HAS_PROCESS( asyncproc );

  sc_in<bool> clock;

  asyncproc(sc_module_name NAME,
	    sc_signal_in_if<bool>& CLOCK)
  {
    clock(CLOCK);
    SC_THREAD( entry );
    sensitive << clock.pos();
  }

  void entry()
  {
    wait();
    while (true) {
      if (clock.posedge()) {
	cout << "AsyncProc: Posedge\n";
      }
      else {
	cout << "AsyncProc: ERROR" << endl;
      }
      wait();
    }
  }
};

SC_MODULE( asyncblock )
{
  SC_HAS_PROCESS( asyncblock );

  sc_in<bool> clock;

  asyncblock(sc_module_name NAME,
	     sc_signal_in_if<bool>& CLOCK)
  {
    clock(CLOCK);
    SC_METHOD( entry );
    sensitive << clock.neg();
  }

  void entry()
  {
    if (clock.posedge()) {
      cout << "AsyncBlock: ERROR\n";
    }
    else {
      cout << "AsyncBlock: Negedge" << endl;
    }
  }
};
    

int
sc_main(int ac, char *av[])
{
  sc_clock clock("Clock", 20, SC_NS, 0.5);

  asyncproc P2("P2", clock);
  asyncblock P3("P3", clock);

  sc_start(160, SC_NS);
  return 0;

}
