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

  a2901_alu_inputs.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef A2901_ALU_INPUTS_H
#define A2901_ALU_INPUTS_H

#include "common.h"

SC_MODULE( a2901_alu_inputs )
{
    SC_HAS_PROCESS( a2901_alu_inputs );

    // shared state
    long* RAM;

    // inputs
    const sig9& I;
    const sig4& Aadd;
    const sig4& Badd;
    const sig4& D;
    const sig4& Q;

    // outputs
    sig4& RE;
    sig4& S;
    sig4& A;

    // constructor
    a2901_alu_inputs( sc_module_name,
                      long* RAM_,
                      const sig9& I_,
                      const sig4& Aadd_,
                      const sig4& Badd_,
                      const sig4& D_,
                      const sig4& Q_,
                      sig4&       RE_,
                      sig4&       S_,
                      sig4&       A_ )
    : RAM( RAM_ ),
      I( I_ ),
      Aadd( Aadd_ ),
      Badd( Badd_ ),
      D( D_ ),
      Q( Q_ ),
      RE( RE_ ),
      S( S_ ),
      A( A_ )
    {
        SC_METHOD( entry );
        sensitive << I;
        sensitive << Aadd;
        sensitive << Badd;
        sensitive << D;
        sensitive << Q;
    }

    void entry();
};

#endif

