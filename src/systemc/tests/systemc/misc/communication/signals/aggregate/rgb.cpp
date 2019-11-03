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

  rgb.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"
#include "rgb.h"

void sc_trace(sc_trace_file* tf,const rgb_t& s, const std::string& NAME) {
  sc_trace(tf, s.red, NAME + ".red");
  sc_trace(tf, s.green, NAME + ".green");
  sc_trace(tf, s.blue, NAME + ".blue");
}

void
some_process::entry()
{
    rgb_t clin;
    rgb_t clout;
  
    while( true ) {
        clin = color_in.read();
        clout = clin;
        clout.red >>= 1;
        clout.green >>= 2;
        clout.blue <<= 1;
        color_out.write( clout );
        wait();
    }
}

int
sc_main( int, char*[] )
{
    sc_signal<rgb_t> in;
    sc_signal<rgb_t> out;

    sc_clock clk( "CLK" );

    some_process foo( "FOO", clk, in, out );
  
    sc_start( 1000, SC_NS );

    return 0;
}
