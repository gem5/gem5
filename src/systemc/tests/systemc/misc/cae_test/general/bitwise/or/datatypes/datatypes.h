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

  datatypes.h -- 

  Original Author: Rocco Jonack, Synopsys, Inc., 1999-07-30

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/


#include "common.h"

SC_MODULE( datatypes )
{
    SC_HAS_PROCESS( datatypes );

    sc_in_clk clk;

    //====================================================================
    // [C] Always Needed Member Function
    //        --  constructor
    //        --  entry
    //====================================================================

    const sc_signal<bool>&             reset ;
    const sc_signal_bool_vector&       in_value1;     // Input  port
    const sc_signal_bool_vector&       in_value2;     // Input  port
    const sc_signal<long>&             in_value3;                        // Input  port
    const sc_signal<int>&              in_value4;                        // Input  port
    const sc_signal<short>&            in_value5;                        // Input  port
    const sc_signal<char>&             in_value6;                        // Input  port
    const sc_signal<char>&             in_value7;                        // Input  port
    const sc_signal<bool>&             in_valid;                         // Input  port
    sc_signal<bool>&                   out_ack;                          // Output port
    sc_signal_bool_vector&             out_value1;    // Output port
    sc_signal_bool_vector&             out_value2;    // Output port
    sc_signal<long>&                   out_value3;                       // Output port
    sc_signal<int>&                    out_value4;                       // Output port
    sc_signal<short>&                  out_value5;                       // Output port
    sc_signal<char>&                   out_value6;                       // Output port
    sc_signal<bool>&                   out_valid;                        // Output port

    //
    // Constructor
    //

    datatypes(
        sc_module_name          NAME,          // referense name
        sc_clock&        CLK,                  // clock
        const  sc_signal<bool>& RESET,
        const  sc_signal_bool_vector&             IN_VALUE1,
        const  sc_signal_bool_vector&             IN_VALUE2,
        const sc_signal<long>&                    IN_VALUE3,
        const sc_signal<int>&                     IN_VALUE4,
        const sc_signal<short>&                   IN_VALUE5,
        const sc_signal<char>&                    IN_VALUE6,
        const sc_signal<char>&                    IN_VALUE7,
        const   sc_signal<bool>&                  IN_VALID,

                sc_signal<bool>&                   OUT_ACK,
                sc_signal_bool_vector&             OUT_VALUE1,
                sc_signal_bool_vector&             OUT_VALUE2,
                sc_signal<long>&                   OUT_VALUE3,
                sc_signal<int>&                    OUT_VALUE4,
                sc_signal<short>&                  OUT_VALUE5,
                sc_signal<char>&                   OUT_VALUE6,
                sc_signal<bool>&                   OUT_VALID
        )
        : 
          reset        (RESET),
          in_value1    (IN_VALUE1),
          in_value2    (IN_VALUE2),
          in_value3    (IN_VALUE3),
          in_value4    (IN_VALUE4),
          in_value5    (IN_VALUE5),
          in_value6    (IN_VALUE6),
          in_value7    (IN_VALUE7),
          in_valid     (IN_VALID),
          out_ack      (OUT_ACK),
          out_value1   (OUT_VALUE1),
          out_value2   (OUT_VALUE2),
          out_value3   (OUT_VALUE3),
          out_value4   (OUT_VALUE4),
          out_value5   (OUT_VALUE5),
          out_value6   (OUT_VALUE6),
          out_valid    (OUT_VALID)

    {
      clk          (CLK);
	  SC_CTHREAD( entry, clk.pos() );
      reset_signal_is(reset,true);
    };

//Process Functionality: Described in the member function below
  void entry();
};

// EOF

