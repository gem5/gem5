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

  bv_arith.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/




/* ----- insert bv_arith_bv_arith_40.h ----- */

#include <systemc.h>

SC_MODULE(bv_arith) {

  sc_out<sc_bigint<80> > XX;
  sc_in_clk Clock;
  sc_in<bool> Reset;
  sc_in<bool> eg_enable;
  sc_in<bool> eg_start;
  sc_out<bool> eg_exitc;


  void COMBI();

  SC_CTOR(bv_arith) {
    SC_METHOD(COMBI);
    sensitive  << eg_enable << eg_start;

  }
};

