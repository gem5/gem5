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

  accessor.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename accessor.h */
/* This is the interface file for synchronous process 'accessor' */

#include "common.h"

SC_MODULE( accessor )
{
  SC_HAS_PROCESS( accessor );

  sc_in_clk clk;

  const signal_bool_vector32& datain;     //input
  sc_signal<bool>& chip_select;   //output
  sc_signal<bool>& write_enable;  //output
  signal_bool_vector10& address;    //output
  signal_bool_vector32& dataout;    //output

  //Constructor 
  accessor(sc_module_name NAME,
	   sc_clock& CLK,
	   const signal_bool_vector32& DATAIN,
	   sc_signal<bool>& CHIP_SELECT,
	   sc_signal<bool>& WRITE_ENABLE,
	   signal_bool_vector10& ADDRESS,
	   signal_bool_vector32& DATAOUT)
    : datain(DATAIN), chip_select(CHIP_SELECT), 
      write_enable(WRITE_ENABLE), address(ADDRESS), dataout(DATAOUT)
  {
    clk(CLK);
	SC_CTHREAD( entry, clk.pos() );
  }

  // Process functionality in member function below
  void entry();
};


