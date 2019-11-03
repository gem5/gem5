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

/* Main routine for biquad simulation */

#include "testbench.h"
#include "biquad.h"
#include "delay_line.h"

float signal_freq;

int
sc_main(int ac, char *av[])
{
  sc_signal<float> sample;
  sc_signal<float> result;
  sc_signal<bool> reset;
  sc_signal<float> delayed_out;

  sample = 0.0;
  result = 0.0;
  reset = false;
  delayed_out = 0.0;

  sc_clock clk("Clock", CLOCK_PERIOD, SC_NS);

  biquad filter("BFILTER", sample, reset, result);
  testbench TB("TB", clk, delayed_out, reset, sample);
  delay_line D("Delay", result, delayed_out, 300);

  int n = 10000;
  signal_freq = 100000;
  for (int i = 1; i < ac; i++) {
    if (av[i][0] == '-') {
      signal_freq = atof(av[i+1]); i++;
      continue;
    }
    n = atoi(av[i]);
  }

  sc_start(n, SC_NS);
  return 0;
}
