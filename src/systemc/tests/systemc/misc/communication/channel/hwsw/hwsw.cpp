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

  hwsw.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

static int randa[] = { 3278, 3420, 1349, 13491, 9234028, 234902,
                       13971, 1498710, 1348179, 1389810, 190101, 92384 };
static int randb[] = { 1349, 20893, 2090092, 8813, 87472, 73231,
                       59413, 42713, 3192, 45653, 565643, 78931, 573231 };
static int randc[] = { 3419, 82093, 9013, 1831, 74372, 233861,
                       421313, 47823, 3902192, 93245653, 77565643, 77234, 10192 };

SC_MODULE( adder_sub )
{
  SC_HAS_PROCESS( adder_sub );

  sc_in_clk clk;

  sc_fifo<int>& Sa; //input
  sc_fifo<int>& Sb; //input
  sc_fifo<int>& Sc; //input
  sc_fifo<int>& Sd; //output
  sc_fifo<int>& Ssum; //output

  // constructor 
  adder_sub( sc_module_name NAME,
	     sc_clock& CLK,
	     sc_fifo<int>& SA,
	     sc_fifo<int>& SB,
	     sc_fifo<int>& SC,
	     sc_fifo<int>& SD,
	     sc_fifo<int>& SSUM )
    : Sa(SA), Sb(SB), Sc(SC), Sd(SD), Ssum(SSUM)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  // Process functionality in member function below
  void entry();
};


int add(int a, int b)
{
  return (a + b);
}

void adder_sub::entry()
{
  int sum;
  int a, b, c, d;

  while (true) {
    // Read inputs
    a = Sa.read();
    b = Sb.read();
    c = Sc.read();
    
    // Perform the computation.
    sum = add(a, b);
    sum = add(sum, c);
    d = a - b;

    // Write outputs
    Ssum.write(sum);
    Sd.write(d);
    // Loop back to do { wait(); } while . 
  }

} // end of entry function

SC_MODULE( testbench )
{
  SC_HAS_PROCESS( testbench );

  sc_in_clk clk;

  sc_fifo<int>& Ssum; //input
  sc_fifo<int>& Sdiff; //input
  sc_fifo<int>& Sa; //output
  sc_fifo<int>& Sb; //output
  sc_fifo<int>& Sc; //output

  // constructor 
  testbench( sc_module_name NAME,
	     sc_clock& CLK,
	     sc_fifo<int>& SSUM,
	     sc_fifo<int>& SDIFF,
	     sc_fifo<int>& SA,
	     sc_fifo<int>& SB,
	     sc_fifo<int>& SC )
    : Ssum(SSUM), Sdiff(SDIFF), Sa(SA), Sb(SB), Sc(SC)
  {
    clk(CLK);
	SC_THREAD( entry );
	sensitive << clk.pos();
  }

  // Process functionality in member function below
  void entry();
};


void testbench::entry()
{
  int a, b, c, d;
  int sum;
  int i;
  char buf[BUFSIZ];

  for (i=0; i < 10; i++) {
    a = randa[i % (sizeof(randa)/sizeof(randa[0]))] & 0x0ff;
    b = randb[i % (sizeof(randb)/sizeof(randb[0]))] & 0x0ff;
    c = randc[i % (sizeof(randc)/sizeof(randc[0]))] & 0x0ff;

    Sa.write(a);
    Sb.write(b);
    Sc.write(c);
    sum = Ssum.read();
    d = Sdiff.read();
    // printf("A = %d, B = %d, C = %d, D = %d, SUM = %d\n", a, b, c, d, sum);
    sprintf(buf, "A = %d, B = %d, C = %d, D = %d, SUM = %d\n", a, b, c, d, sum);
    cout << buf;
  }
  sc_stop();

} // end of entry function

// Main routine

int sc_main(int ac, char *av[])
{
  sc_fifo<int> a;
  sc_fifo<int> b;
  sc_fifo<int> c;
  sc_fifo<int> d;
  sc_fifo<int> sum;
  sc_clock clock("Clock", 10, SC_NS, 0.5, 0, SC_NS, 0);

  testbench T("TB", clock, sum, d, a, b, c);
  adder_sub AS("AS", clock, a, b, c, d, sum);

  sc_start();
  return 0;
}
