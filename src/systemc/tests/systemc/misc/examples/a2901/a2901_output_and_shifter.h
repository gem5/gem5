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

  a2901_output_and_shifter.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_OUTPUT_AND_SHIFTER_H
#define A2901_OUTPUT_AND_SHIFTER_H

#include "common.h"

SC_MODULE( a2901_output_and_shifter )
{
    SC_HAS_PROCESS( a2901_output_and_shifter );

    // inputs
    const sig9& I;
    const sig1& OEbar;
    const sig4& A;
    const sig4& F;
    const sig4& Q;

    // outputs
    sig4& Y;
    sig1& t_RAM0;
    sig1& t_RAM3;
    sig1& t_Q0;
    sig1& t_Q3;

    // constructor
    a2901_output_and_shifter( sc_module_name,
                              const sig9& I_,
                              const sig1& OEbar_,
                              const sig4& A_,
                              const sig4& F_,
                              const sig4& Q_,
                              sig4&       Y_,
                              sig1&       t_RAM0_,
                              sig1&       t_RAM3_,
                              sig1&       t_Q0_,
                              sig1&       t_Q3_ )
    : I( I_ ),
      OEbar( OEbar_ ),
      A( A_ ),
      F( F_ ),
      Q( Q_ ),
      Y( Y_ ),
      t_RAM0( t_RAM0_ ),
      t_RAM3( t_RAM3_ ),
      t_Q0( t_Q0_ ),
      t_Q3( t_Q3_ )
    {
        SC_METHOD( entry );
        sensitive << I;
        sensitive << OEbar;
        sensitive << A;
        sensitive << F;
        sensitive << Q;
    }

    void entry();
};

#endif

