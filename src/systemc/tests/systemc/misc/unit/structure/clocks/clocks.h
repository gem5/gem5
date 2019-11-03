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

  clocks.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

/******************************************************************************/
/***************************    CLK_POS Function         **********************/
/******************************************************************************/

SC_MODULE( CLK_POS )
{
    SC_HAS_PROCESS( CLK_POS );

    sc_in_clk clk;

    sc_signal<bool>&	out_clk_pos; 
 
    CLK_POS ( sc_module_name   	NAME,
              sc_clock&		TICK_P,
	      sc_signal<bool>&	OUT_CLK_POS )
 
    : 
      out_clk_pos (OUT_CLK_POS)
    {
        clk (TICK_P);
	SC_CTHREAD( entry, clk.pos() );
    }
 
  void entry();
};

void
CLK_POS::entry()
{
  cout << sc_time_stamp() << " : CLK UP\n" << endl;
  out_clk_pos.write(1);
  wait();
  cout << sc_time_stamp() << " : CLK UP\n" << endl;
  out_clk_pos.write(0);
  wait();
  cout << sc_time_stamp() << " : CLK UP\n" << endl;
  wait();
  cout << sc_time_stamp() << " : CLK UP\n" << endl;
}

/******************************************************************************/
/***************************    CLK_NEG Function         **********************/
/******************************************************************************/

SC_MODULE( CLK_NEG )
{
    SC_HAS_PROCESS( CLK_NEG );

    sc_in_clk clk;

    sc_signal<bool>&	out_clk_neg; 
 
    CLK_NEG ( sc_module_name   	NAME,
              sc_clock&		TICK_N,
	      sc_signal<bool>&	OUT_CLK_NEG )
    : 
      out_clk_neg (OUT_CLK_NEG)
    {
        clk (TICK_N);
	SC_CTHREAD( entry, clk.neg() );
    }
 
  void entry();
};

void
CLK_NEG::entry()
{
  cout << sc_time_stamp() << " : CLK DN\n" << endl;
  out_clk_neg.write(1);
  wait();
  cout << sc_time_stamp() << " : CLK DN\n" << endl;
  out_clk_neg.write(0);
  wait();
  cout << sc_time_stamp() << " : CLK DN\n" << endl;
  wait();
  cout << sc_time_stamp() << " : CLK DN\n" << endl;
}

/******************************************************************************/
/***************************    CLK2_POS Function        **********************/
/******************************************************************************/

SC_MODULE( CLK2_POS )
{
    SC_HAS_PROCESS( CLK2_POS );

    sc_in_clk clk;

    sc_signal<bool>&	out_clk2_pos; 
 
    CLK2_POS ( sc_module_name  	NAME,
               sc_clock&	TICK2_P,
	       sc_signal<bool>&	OUT_CLK2_POS )
    : 
      out_clk2_pos (OUT_CLK2_POS)
    {
        clk (TICK2_P);
	SC_CTHREAD( entry, clk.pos() );
    }
 
  void entry();
};

void
CLK2_POS::entry()
{
  cout << sc_time_stamp() << " : _____________CLK2 UP\n" << endl;
  out_clk2_pos.write(1);
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 UP\n" << endl;
  out_clk2_pos.write(0);
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 UP\n" << endl;
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 UP\n" << endl;
}

/******************************************************************************/
/***************************    CLK2_NEG Function        **********************/
/******************************************************************************/

SC_MODULE( CLK2_NEG )
{
    SC_HAS_PROCESS( CLK2_NEG );

    sc_in_clk clk;

    sc_signal<bool>&	out_clk2_neg; 
 
    CLK2_NEG ( sc_module_name  	NAME,
               sc_clock&	TICK2_N,
	       sc_signal<bool>&	OUT_CLK2_NEG )
    : 
      out_clk2_neg (OUT_CLK2_NEG)
    {
        clk (TICK2_N);
	SC_CTHREAD( entry, clk.neg() );
    }
 
  void entry();
};

void
CLK2_NEG::entry()
{
  cout << sc_time_stamp() << " : _____________CLK2 DN\n" << endl;
  out_clk2_neg.write(1);
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 DN\n" << endl;
  out_clk2_neg.write(0);
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 DN\n" << endl;
  wait();
  cout << sc_time_stamp() << " : _____________CLK2 DN\n" << endl;
}
