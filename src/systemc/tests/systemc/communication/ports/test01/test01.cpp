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

// test of named ports

#include "systemc.h"

SC_MODULE( mod_a )
{
    sc_in_clk    in_clk;
    sc_inout_clk inout_clk;
    sc_out_clk   out_clk;

    sc_fifo_in<int>  fifo_in;
    sc_fifo_out<int> fifo_out;

    sc_port<sc_signal_in_if<float> > port;

    sc_in<int>         in_int;
    sc_in<bool>        in_bool;
    sc_in<sc_logic>    in_logic;
    sc_inout<int>      inout_int;
    sc_inout<bool>     inout_bool;
    sc_inout<sc_logic> inout_logic;
    sc_out<int>        out_int;
    sc_out<bool>       out_bool;
    sc_out<sc_logic>   out_logic;

    sc_in_resolved    in_resolved;
    sc_inout_resolved inout_resolved;
    sc_out_resolved   out_resolved;

    sc_in_rv<1>    in_rv;
    sc_inout_rv<1> inout_rv;
    sc_out_rv<1>   out_rv;

    SC_CTOR( mod_a )
    : in_clk( "in_clk" ), inout_clk( "inout_clk" ), out_clk( "out_clk" ),
      fifo_in( "fifo_in" ), fifo_out( "fifo_out" ),
      port( "port" ),
      in_int( "in_int" ), in_bool( "in_bool" ), in_logic( "in_logic" ),
      inout_int( "inout_int" ), inout_bool( "inout_bool" ),
      inout_logic( "inout_logic" ),
      out_int( "out_int" ), out_bool( "out_bool" ), out_logic( "out_logic" ),
      in_resolved( "in_resolved" ), inout_resolved( "inout_resolved" ),
      out_resolved( "out_resolved" ),
      in_rv( "in_rv" ), inout_rv( "inout_rv" ), out_rv( "out_rv" )
    {}
};

#define WRITE(a) \
    cout << a.name() << " (" << a.kind() << ")" << endl

int
sc_main( int, char*[] )
{
    mod_a a( "a" );

    WRITE( a.in_clk );
    WRITE( a.inout_clk );
    WRITE( a.out_clk );

    WRITE( a.fifo_in );
    WRITE( a.fifo_out );

    WRITE( a.port );

    WRITE( a.in_int );
    WRITE( a.in_bool );
    WRITE( a.in_logic );
    WRITE( a.inout_int );
    WRITE( a.inout_bool );
    WRITE( a.inout_logic );
    WRITE( a.out_int );
    WRITE( a.out_bool );
    WRITE( a.out_logic );

    WRITE( a.in_resolved );
    WRITE( a.inout_resolved );
    WRITE( a.out_resolved );

    WRITE( a.in_rv );
    WRITE( a.inout_rv );
    WRITE( a.out_rv );

    return 0;
}
