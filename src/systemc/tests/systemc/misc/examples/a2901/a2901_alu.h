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

  a2901_alu.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_ALU_H
#define A2901_ALU_H

#include "common.h"

SC_MODULE( a2901_alu )
{
    SC_HAS_PROCESS( a2901_alu );

    // inputs
    const sig9& I;
    const sig4& RE;
    const sig4& S;
    const sig1& C0;

    // outputs
    sig5& R_ext;
    sig5& S_ext;
    sig4& F;
    sig1& OVR;
    sig1& C4;
    sig1& Pbar;
    sig1& Gbar;
    sig1& F3;
    sig1& F30;

    // temporaries
    int5 result;
    int5 R_ext_v;
    int5 S_ext_v;
    int5 temp_p;
    int5 temp_g;

    // constructor
    a2901_alu( sc_module_name,
               const sig9& I_,
               const sig4& RE_,
               const sig4& S_,
               const sig1& C0_,
               sig5&       R_ext_,
               sig5&       S_ext_,
               sig4&       F_,
               sig1&       OVR_,
               sig1&       C4_,
               sig1&       Pbar_,
               sig1&       Gbar_,
               sig1&       F3_,
               sig1&       F30_ )
    : I( I_ ),
      RE( RE_ ),
      S( S_ ),
      C0( C0_ ),
      R_ext( R_ext_ ),
      S_ext( S_ext_ ),
      F( F_ ),
      OVR( OVR_ ),
      C4( C4_ ),
      Pbar( Pbar_ ),
      Gbar( Gbar_ ),
      F3( F3_ ),
      F30( F30_ )
    {
        SC_METHOD( entry );
        sensitive << I;
        sensitive << RE;
        sensitive << S;
        sensitive << C0;
    }

    void entry();
};

#endif

