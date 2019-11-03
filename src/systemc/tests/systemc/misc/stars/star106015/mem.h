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

  mem.h -- 

  Original Author: Stan Liao, Synopsys, Inc., 2000-07-11

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

struct memory : sc_module {

  sc_in_clk            clk;
  sc_in<sc_uint<8> >   in_value1; 
  sc_in<bool>          in_valid1;     
  sc_out<bool>         out_valid1;    

  SC_HAS_PROCESS( memory );

  memory (const  char    *NAME)
        : sc_module (NAME)    
    {
      SC_CTHREAD( entry, clk.pos() );
      end_module();
    };

    void entry ();

};

