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

  arraytypes.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "arr_struct.h"

SC_MODULE( arraytypes ) {
  sc_in_clk clk;
  sc_in<bool>            reset;
  sc_in<arr_struct1>             in_value1;     
  sc_in<sc_lv<1> >              in_valid;      
  sc_out<arr_struct2>                   out_value1;    
  sc_out<sc_bit>                    out_valid;     

    SC_CTOR(arraytypes)
      {
	SC_CTHREAD (entry, clk.pos());
	reset_signal_is( reset, true );
      };
      
    void entry ();

};

