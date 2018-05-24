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

//
// Demo Code for SystemC 2.0 Casting Problem
//
// Rob Slater
// Motorola, Inc.
// r.slater@motorola.com
//
//
// Compile this file with:
//   g++ -Wall
// (use g++ version 2.95.2)
//
// With SystemC 1.0.2 the file compiles and prints
// "a = 10" to stdout.
//
// With SystemC 2.0 (release) line 32 fails compilation
// with the error:
//   sc_main.cc:32: `const class sc_uint<16>' used where a `unsigned int' was expected
//

#include <systemc.h>

// sc_uint<16> unused;

SC_MODULE(Test)
{
  sc_in_clk clk;
  sc_in<sc_uint<16> > in;

  void meth()
  {
    unsigned int a;

    a = in.read();
    cout << "a = " << a << endl;

  }  // meth()

  SC_CTOR(Test)
  {
    SC_METHOD(meth);
    sensitive << clk.pos();

  }  // SC_CTOR(Test)

};  // SC_MODULE(Test)


int sc_main(int argc, char *argv[])
{
  // Declare the clock
  sc_clock clk("clk", 50, SC_NS, 0.5, 0, SC_NS, false);

  sc_signal<sc_uint<16> > in;

  Test test("test");
  test.clk(clk);
  test.in(in);

  in.write(10);  // Initialize "in" with "10"
  sc_start(75, SC_NS);  // Run for 1-1/2 clock cycles
  
  return 0;

}  // sc_main()
