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

  test01.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
Martin,
tis is a very simple test that I used to verify functionality and timing
with 1.2.1 on NT and I can see how memory consumption grows linearly with
time. When I ran my implementation I could see the program footprint of
about 400K during the whole simulation.
The test runs about 20 seconds and memory footprint grows to about 90MB. If
you run it longer it starts swapping memory do the disk and goes very
slowly. So I waited till it grew to 1GB and then killed it because all
computer does at that point is swapping memory. I guess if you let it run
long enough it would eventually die. I think this happens only with
optimized libraries.
*/

//--------------------------------------------------------------------------

#include <systemc.h>

#define sc_ns SC_NS
#define sc_ms SC_MS

class A : public sc_module
{
  sc_event e1;

  void f()
  {
    while(true)
    {
      e1.notify(sc_time(5,sc_ns));
      wait(e1);

      wait(2,sc_ns);
    }
  }
  public:
  sc_in_clk clk;
  SC_CTOR(A) { SC_THREAD(f); sensitive<<clk;}
};

class B : public sc_module
{
  sc_event e2, e3;
  void f()
  {
    while(true)
    {
      e2.notify(sc_time(1,sc_ns));
      wait(e2);

      e3.notify(sc_time(10,sc_ns));

      wait(e3);
    }
  }
  public:
  sc_in_clk clk;
  SC_CTOR(B) { SC_THREAD(f); sensitive<<clk;}
};

int sc_main(int argc, char* argv[])
{
  A a("a");
  B b("b");

 sc_time clk_time(10.0, sc_ns);
 sc_clock clock("clock",clk_time);
 a.clk(clock);
 b.clk(clock);

  sc_start(0, SC_NS);
  sc_time sim_time(10.,sc_ms);
  sc_start(sim_time);
  return 0;
}
//--------------------------------------------------------------------------
