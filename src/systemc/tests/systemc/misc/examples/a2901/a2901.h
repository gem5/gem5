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

  a2901.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_H
#define A2901_H

#include "a2901_alu.h"
#include "a2901_alu_inputs.h"
#include "a2901_output_and_shifter.h"
#include "a2901_edge.h"

SC_MODULE( a2901 )
{
    // shared state
    long RAM[15];

    // signals
    sig4 RE, S, F, Q, A;
    sig5 R_ext, S_ext;

    // constructor
    a2901( sc_module_name,
           const sc_clock& CLK_,
           const sig9& I_,
           const sig4& Aadd_,
           const sig4& Badd_,
           const sig4& D_,
           const sig1& RAM0_,
           const sig1& RAM3_,
           const sig1& Q0_,
           const sig1& Q3_,
           const sig1& C0_,
           const sig1& OEbar_,
           sig4&       Y_,
           sig1&       t_RAM0_,
           sig1&       t_RAM3_,
           sig1&       t_Q0_,
           sig1&       t_Q3_,
           sig1&       C4_,
           sig1&       Gbar_,
           sig1&       Pbar_,
           sig1&       OVR_,
           sig1&       F3_,
           sig1&       F30_ )
    {
        SC_NEW( a2901_alu( "alu",
                           I_, RE, S, C0_,
                           R_ext, S_ext, F, OVR_, C4_, Pbar_, Gbar_,
                           F3_, F30_ ) );
        SC_NEW( a2901_alu_inputs( "alu_inputs",
                                  RAM,
                                  I_, Aadd_, Badd_, D_, Q,
                                  RE, S, A ) );
        SC_NEW( a2901_output_and_shifter( "o_and_s",
                                           I_, OEbar_, A, F, Q,
                                           Y_, t_RAM0_, t_RAM3_,
                                           t_Q0_, t_Q3_ ) );
        SC_NEW( a2901_edge( "edge",
                            CLK_,
                            RAM,
                            I_, Badd_, F, Q3_, Q0_, RAM3_, RAM0_,
                            Q ) );
        // initialize the RAM (to get rid of UMRs)
        for( int i = 0; i < 15; ++ i ) {
            RAM[0] = 0;
        }
    }
};

#endif

