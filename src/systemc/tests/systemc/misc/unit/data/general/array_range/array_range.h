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

  array_range.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

                /**************************************/
                /* Interface Filename:	array_range.h   */
                /**************************************/

#include "common.h"

SC_MODULE( array_range )
{
  SC_HAS_PROCESS( array_range );

  sc_in_clk clk;

  // Inputs
	const signal_bool_vector8&	in1;
  // Outputs
	signal_bool_vector4&		o1;
	signal_bool_vector4&		o2;
	signal_bool_vector8&		o3;
	signal_bool_vector8&		o4;
	signal_bool_vector8&		o5;

  // Constructor
  array_range (sc_module_name 		NAME,
	sc_clock&			TICK,
	const signal_bool_vector8&	IN1,
	signal_bool_vector4&		O1,
	signal_bool_vector4&		O2,
	signal_bool_vector8&		O3,
	signal_bool_vector8&		O4,
	signal_bool_vector8&		O5 )
	
      : 
	in1	(IN1),
	o1	(O1),
	o2	(O2),
	o3	(O3),
	o4	(O4),
	o5	(O5)

  	{
          clk	(TICK);
	  SC_CTHREAD( entry, clk.pos() );
        }

  void entry();
};
