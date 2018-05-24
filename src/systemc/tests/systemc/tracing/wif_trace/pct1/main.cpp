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

  main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Main routine for tri-state simulation */

#include "tx.h"
#include "monitor.h"

int sc_main(int ac, char *av[])
{
  sc_signal<bool> tx;
  sc_signal<bool> wr;
  sc_signal<int>  dout;
  sc_signal<bool> two_stop_bits;
  sc_clock clock("CLK", 10.0, SC_NS, 0.5, 0.0, SC_NS);
  sc_trace_file *tf = sc_create_wif_trace_file("pct1");

  tx = true;
  wr = true;
  dout = 0;

  two_stop_bits = true;
  sio_tx TX("Transmitter", clock, two_stop_bits, tx);
  monitor MON("Monitor", clock, tx, dout, wr);

  //  sc_trace(tf, clock.signal(), "Clock");
  sc_trace(tf, tx, "Tx");
  sc_trace(tf, dout, "dout");
  sc_trace(tf, wr, "wr");

  sc_start(500, SC_NS);
  sc_close_wif_trace_file( tf );
  return 0;
}

