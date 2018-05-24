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

  pipeline.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename pipeline.h */
/* This is the interface file for module `pipeline' */

#include "systemc.h"

struct pipeline : public sc_module { 
  sc_signal<double> prod;
  sc_signal<double> quot;

  stage1_2 S1_2; // component

  //Constructor 
  pipeline(sc_module_name NAME,
	   sc_clock& CLK,
	   const sc_signal<double>& IN1,
	   const sc_signal<double>& IN2,
	   sc_signal<double>& OUT_)
    : sc_module(NAME), 
      S1_2("Stage1_2", CLK, IN1, IN2, prod, quot)
  {
    f_stage3("Stage3", CLK, prod, quot, OUT_);
  }
};


