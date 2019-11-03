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

  datawidth.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /**************************************/
                /* Interface Filename:	datawidth.h   */
                /**************************************/

#include "common.h"

SC_MODULE( datawidth )
{
    SC_HAS_PROCESS( datawidth );

    sc_in_clk clk;

  // Inputs
	const signal_bool_vector6&	in1;
	const signal_bool_vector6&	in2;
	const sc_signal<bool>&		ready;
  // Outputs
	signal_bool_vector7&		result;
  // Parameters
	const int 			in1_width;
	const int 			in2_width;
	const int 			result_width;

  // Constructor
  datawidth (sc_module_name 		NAME,
	sc_clock&			TICK,
	const signal_bool_vector6&	IN1,
	const signal_bool_vector6&	IN2,
	const sc_signal<bool>&		READY,
	signal_bool_vector7&		RESULT,
	const int			IN1_WIDTH,
	const int                       IN2_WIDTH,
	const int                       RESULT_WIDTH )
	
      : 
	in1	(IN1),
	in2	(IN2),
	ready   (READY),
	result	(RESULT),
	in1_width (IN1_WIDTH),
	in2_width (IN2_WIDTH),
	result_width (RESULT_WIDTH)

  	{
            clk	(TICK);
	    SC_CTHREAD( entry, clk.pos() );
        }

  void entry();
};
