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

  stimulus.h -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-14

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "common.h"

SC_MODULE( stimulus )
{
    SC_HAS_PROCESS( stimulus );

    sc_in_clk clk;

    sc_signal<bool>&             reset;
    sc_signal_bool_vector&       out_value1;     // Output  port
    sc_signal_bool_vector&       out_value2;     // Output  port
    sc_signal<long>&             out_value3;                        // Output  port
    sc_signal<int>&              out_value4;                        // Output  port
    sc_signal<short>&            out_value5;                        // Output  port
    sc_signal<char>&             out_value6;                        // Output  port
    sc_signal<bool>&             out_valid;                         // Output  port
    const sc_signal<bool>&       in_ack;

    //
    // Constructor
    //

    stimulus(
        sc_module_name    NAME,      // reference name
        sc_clock&      CLK,          // clock
        sc_signal<bool>& RESET,
        sc_signal_bool_vector&             OUT_VALUE1,
        sc_signal_bool_vector&             OUT_VALUE2,
        sc_signal<long>&                   OUT_VALUE3,
        sc_signal<int>&                    OUT_VALUE4,
        sc_signal<short>&                  OUT_VALUE5,
        sc_signal<char>&                   OUT_VALUE6,
        sc_signal<bool>&                   OUT_VALID,
        const sc_signal<bool>&             IN_ACK
        )
        : 
          reset        (RESET),
          out_value1    (OUT_VALUE1),
          out_value2    (OUT_VALUE2),
          out_value3    (OUT_VALUE3),
          out_value4    (OUT_VALUE4),
          out_value5    (OUT_VALUE5),
          out_value6    (OUT_VALUE6),
          out_valid     (OUT_VALID),
          in_ack        (IN_ACK) 
   {
     clk          (CLK);
	 SC_CTHREAD( entry, clk.pos() );
   };
  void entry();
};
// EOF
