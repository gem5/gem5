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

  a2901_edge.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_EDGE_H
#define A2901_EDGE_H

#include "common.h"

SC_MODULE( a2901_edge )
{
    SC_HAS_PROCESS( a2901_edge );

    // clock
    const sc_clock& CLK;

    // shared state
    long* RAM;

    // inputs
    const sig9& I;
    const sig4& Badd;
    const sig4& F;
    const sig1& Q3;
    const sig1& Q0;
    const sig1& RAM3;
    const sig1& RAM0;

    // outputs
    sig4& Q;

    // temporaries
    sc_uint<3> i86;
    sc_uint<3> i87;
    sc_uint<3> q31, q20;
    sc_uint<3> f31, f20;

    // constructor
    a2901_edge( sc_module_name,
                const sc_clock& CLK_,
                long*           RAM_,
                const sig9&     I_,
                const sig4&     Badd_,
                const sig4&     F_,
                const sig1&     Q3_,
                const sig1&     Q0_,
                const sig1&     RAM3_,
                const sig1&     RAM0_,
                sig4&           Q_ )
    : CLK( CLK_ ),
      RAM( RAM_ ),
      I( I_ ),
      Badd( Badd_ ),
      F( F_ ),
      Q3( Q3_ ),
      Q0( Q0_ ),
      RAM3( RAM3_ ),
      RAM0( RAM0_ ),
      Q( Q_ )
    {
        SC_METHOD( entry );
        sensitive << CLK.posedge_event();
    }

    void entry();
};

#endif

