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

  a2901_test.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_TEST_H
#define A2901_TEST_H

#include "common.h"

SC_MODULE( a2901_test )
{
    SC_HAS_PROCESS( a2901_test );

    // clock
    const sc_clock& CLK;

    // inputs
    const sig4& Y;
    const sig1& t_RAM0;
    const sig1& t_RAM3;
    const sig1& t_Q0;
    const sig1& t_Q3;
    const sig1& C4;
    const sig1& Gbar;
    const sig1& Pbar;
    const sig1& OVR;
    const sig1& F3;
    const sig1& F30;

    // outputs
    sig9& I;
    sig4& Aadd;
    sig4& Badd;
    sig4& D;
    sig1& RAM0;
    sig1& RAM3;
    sig1& Q0;
    sig1& Q3;
    sig1& C0;
    sig1& OEbar;

    // temporaries
    int vec_cnt;
    int loop;

    // constructor
    a2901_test( sc_module_name,
                const sc_clock& CLK_,
                const sig4& Y_,
                const sig1& t_RAM0_,
                const sig1& t_RAM3_,
                const sig1& t_Q0_,
                const sig1& t_Q3_,
                const sig1& C4_,
                const sig1& Gbar_,
                const sig1& Pbar_,
                const sig1& OVR_,
                const sig1& F3_,
                const sig1& F30_,
                sig9&       I_,
                sig4&       Aadd_,
                sig4&       Badd_,
                sig4&       D_,
                sig1&       RAM0_,
                sig1&       RAM3_,
                sig1&       Q0_,
                sig1&       Q3_,
                sig1&       C0_,
                sig1&       OEbar_ )
    : CLK( CLK_ ),
      Y( Y_ ),
      t_RAM0( t_RAM0_ ),
      t_RAM3( t_RAM3_ ),
      t_Q0( t_Q0_ ),
      t_Q3( t_Q3_ ),
      C4( C4_ ),
      Gbar( Gbar_ ),
      Pbar( Pbar_ ),
      OVR( OVR_ ),
      F3( F3_ ),
      F30( F30_ ),
      I( I_ ),
      Aadd( Aadd_ ),
      Badd( Badd_ ),
      D( D_ ),
      RAM0( RAM0_ ),
      RAM3( RAM3_ ),
      Q0( Q0_ ),
      Q3( Q3_ ),
      C0( C0_ ),
      OEbar( OEbar_ )
    {
        vec_cnt = 0;
        loop = 0;

        // init
        I.write( 0x7 );
        D.write( 0 );
        C0.write( 0 );
        OEbar.write( 0 );
        Aadd.write( 0 );
        Badd.write( 0 );
        Q0.write( 0 );
        Q3.write( 0 );

        SC_METHOD( entry );
        sensitive << CLK.posedge_event();
    }

    void entry();
};

#endif
