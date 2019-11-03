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

  resolved_sig.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

                /**************************************/

SC_MODULE( stimgen )
{
    SC_HAS_PROCESS( stimgen );

    // Inputs
    sc_in_clk  clk;
    sc_in<int> result;

    // Outputs
    sc_out<int>  in1;
    sc_out<int>  in2;
    sc_out<bool> ready;
    sc_out_rv<8> bus;

    // Constructor
    stimgen( sc_module_name        NAME,
	     sc_clock&             TICK,
	     sc_signal<int>& RESULT,
	     sc_signal<int>&       IN1,
	     sc_signal<int>&       IN2,
	     sc_signal<bool>&      READY,
	     sc_signal_rv<8>&      BUS )
    {
	SC_CTHREAD( entry, clk.pos() );
        clk( TICK );
        result( RESULT );
        in1( IN1 );
        in2( IN2 );
        ready( READY );
		bus( BUS );
    }

    void entry();
};

                /**************************************/

SC_MODULE( datawidth )
{
    SC_HAS_PROCESS( datawidth );

    // Inputs
    sc_in_clk      clk;
    sc_in<int>     in1;
    sc_in<int>     in2;
    sc_in<bool>    ready;
    sc_inout_rv<8> bus;

    // Outputs
    sc_out<int> result;

    // Constructor
    datawidth( sc_module_name         NAME,
	       sc_clock&              TICK,
	       sc_signal<int>&  IN1,
	       sc_signal<int>&  IN2,
	       sc_signal<bool>& READY,
	       sc_signal<int>&        RESULT,
	       sc_signal_rv<8>&       BUS )
    {
	SC_CTHREAD( entry, clk.pos() );
        clk( TICK );
        in1( IN1 );
        in2( IN2 );
        ready( READY );
        bus( BUS );
        result( RESULT );
    }

    void entry();
};

                /*******************************************/
 
void
datawidth::entry()
{
    int tmp_a;
    int	tmp_b;
    int tmp_result;

    while (true) {
    
	// HANDSHAKING
	do { wait(); } while ( ready != 1 );

	// COMPUTATION
	tmp_a = in1.read();
	tmp_b = in2.read();
	tmp_result = tmp_a + tmp_b;
	cout << ";    reading from bus=" << bus.read() << endl;
	sc_lv<8> lv( '0' );
	lv[1] = lv[3] = lv[5] = lv[7] = 1;
	cout << "datawidth writing to bus: " << lv << endl;
	bus.write( lv );

	// WRITE OUTPUT
	result.write( tmp_result ); // result = in1 + in2
	wait();
    }
}

                /*****************************************/
 
void
stimgen::entry()
{
    int i;
    int j;

    ready.write( 0 );

    for( i = 0; i < 6; i ++ ) { // integer in1 (6 bits of data)
	for( j = 0; j < 6; j ++ ) { // integer in2 (6 bits of data)
	    in1.write( i );
	    in2.write( j );
	    ready.write( 1 );
	    sc_lv<8> lv( sc_logic( i % 4 ) );
	    lv[1] = lv[3] = lv[5] = lv[7] = j % 4;
	    cout << "writing to bus: " << lv << flush;
	    bus.write( lv );
	    wait();

	    ready.write( 0 );
	    wait();

	    // cout << in1.read() << " + " << in2.read()
	    //	    << " = " << result.read() << endl;
	}
    }

    sc_stop();
}

                /***************************************/
		/*				       */
		/*	int = int + int		       */
                /*                                     */
                /*      Max addition is 63 + 63        */
		/*				       */
                /***************************************/
 
int
sc_main( int ac, char *av[] )
{
    // Signal Instantiation
    sc_signal<int> in1( "in1" );
    sc_signal<int> in2( "in2" );
    sc_signal<int> result( "result" );
    sc_signal<bool> ready( "ready" );
    sc_signal_rv<8> bus( "bus" );

    // Clock Instantiation
    sc_clock clk( "clock", 10, SC_NS, 0.5, 0, SC_NS );

    // Process Instantiation
    datawidth D1( "D1", clk, in1, in2, ready, result, bus );
    stimgen T1( "T1", clk, result, in1, in2, ready,bus );

    // Simulation Run Control
    sc_start();

    return 0;
}
