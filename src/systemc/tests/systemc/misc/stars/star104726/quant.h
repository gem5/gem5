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

  quant.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef QUANTH
#define QUANTH

#include <systemc.h>
#include "global.h"

SC_MODULE(quant) {

  sc_in_clk        clk;

  sc_in<Coeff8x8>  data_in; 
  sc_in<bool>      start;
  sc_in<bool>      data_ok;

  sc_out<Coeff8x8> data_out;
  sc_out<bool>     ready;
  sc_out<bool>     data_out_ready;    

  void do_quant();

  SC_CTOR(quant) {
    SC_CTHREAD(do_quant,clk.pos());
  };

};


#endif

