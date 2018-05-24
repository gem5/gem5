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

  tb.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "common.h"

SC_MODULE( testbench )
{
    SC_HAS_PROCESS( testbench );

    sc_in_clk clk;

    sc_out<bool>       reset;
    sc_out<bool>       x_ok;
    sc_out<bool>       y_ok;
    sc_in<bool>        data_ready;
    sc_in<bool>        select_xy;
    sc_in<bool_vector> coord_xy;

    void entry();
 
    testbench( sc_module_name            name_,
	       const sc_clock&           clk_,
	       sc_signal<bool>&          reset_,
	       sc_signal<bool>&          x_ok_,
	       sc_signal<bool>&          y_ok_,
	       const sc_signal<bool>&    data_ready_,
	       const sc_signal<bool>&    select_xy_,
	       const signal_bool_vector& coord_xy_ )
        : sc_module( name_ )
    {
          clk( clk_ );
	  reset( reset_ );
	  x_ok( x_ok_ );
	  y_ok( y_ok_ );
	  data_ready( data_ready_ );
	  select_xy( select_xy_ );
	  coord_xy( coord_xy_ );
	SC_CTHREAD( entry, clk.neg() );
    }
};

sc_bv<16> mem[17];

void
testbench::entry()
{
    bool_vector	x_coord;
    bool_vector	y_coord;
    int x_flag = 0;
    int y_flag = 0;
    int i;		// Counter variable
    int x = 0;		// Memory location of x_coord
    int y = 1;		// Memory location of y_coord
    
    // reset initialization
    
    reset.write(1);
    x_ok.write(0);
    y_ok.write(0);
    wait();
    reset.write(0);
    wait();

    // fill display memory with zeros

    for (i = 1; i < 17; i++)
	mem[i] = 0;

    // capture of (x,y) coordinates

    while(true) {

        // wait for new x or y coordinate to be calculated

	do { wait(); } while (data_ready == 0);

        // capture x coordinate

	if(select_xy.read() == 0) {
	    x_coord = coord_xy.read(); 
	    x_flag = x_flag + 1;
	    x_ok.write(1);
	}

        // capture y coordinate

	if(select_xy.read() == 1) {
	    y_coord = coord_xy.read(); 
	    y_flag = y_flag + 1;
	    y_ok.write(1);
	}

	wait();
	x_ok.write(0);
	y_ok.write(0);

        // debug display of coordinate sets
        /*
	if (x_flag == y_flag) {
	    cout << " Coordinate Set #" << x_flag
	         << " X = " << x_coord.to_int() 
	         << " Y = " << y_coord.to_int()
	         << endl;
	}
        */

        // conversion of x coordinate values to memory column locations

	if (x_coord.to_int() == -8) x = 15;
	if (x_coord.to_int() == -7) x = 14;
	if (x_coord.to_int() == -6) x = 13;
	if (x_coord.to_int() == -5) x = 12;
	if (x_coord.to_int() == -4) x = 11;
	if (x_coord.to_int() == -3) x = 10;
	if (x_coord.to_int() == -2) x = 9;
	if (x_coord.to_int() == -1) x = 8;
	if (x_coord.to_int() == 0) x = 7;
	if (x_coord.to_int() == 1) x = 6;
	if (x_coord.to_int() == 2) x = 5;
	if (x_coord.to_int() == 3) x = 4;
	if (x_coord.to_int() == 4) x = 3;
	if (x_coord.to_int() == 5) x = 2;
	if (x_coord.to_int() == 6) x = 1;
	if (x_coord.to_int() == 7) x = 0;

        // conversion of y coordinate values to memory row locations

	if (y_coord.to_int() == -8) y = 16;
	if (y_coord.to_int() == -7) y = 15;
	if (y_coord.to_int() == -6) y = 14;
	if (y_coord.to_int() == -5) y = 13;
	if (y_coord.to_int() == -4) y = 12;
	if (y_coord.to_int() == -3) y = 11;
	if (y_coord.to_int() == -2) y = 10;
	if (y_coord.to_int() == -1) y = 9;
	if (y_coord.to_int() == 0) y = 8;
	if (y_coord.to_int() == 1) y = 7;
	if (y_coord.to_int() == 2) y = 6;
	if (y_coord.to_int() == 3) y = 5;
	if (y_coord.to_int() == 4) y = 4;
	if (y_coord.to_int() == 5) y = 3;
	if (y_coord.to_int() == 6) y = 2;
	if (y_coord.to_int() == 7) y = 1;

        // turn bit high in memory for calculated coordinate

	mem[y][x] = 1;

        // stop simulation after 100 coordinates

	if (y_flag == 100) break;

    } // End of while loop

    cout << "\n\t FINAL MEMORY VALUES" << endl;

    for (i = 1; i < 17; i++)
	cout << "Memory Location " << i 
	     << " : \t" << mem[i] << endl;

    sc_stop();
}
