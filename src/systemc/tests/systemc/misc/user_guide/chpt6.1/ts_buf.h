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

  ts_buf.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename ts_buf.h */
/* This is the interface file for asynchronous process `ts_buf' */

#include "common.h"

SC_MODULE( ts_buf )
{
  SC_HAS_PROCESS( ts_buf );

  const sc_signal<bool>& in; //input
  const sc_signal<bool>& control; //input
  signal_std_logic& ts_out; //output

  //Constructor 
  ts_buf(sc_module_name NAME,
	 const sc_signal<bool>& IN_,
	 const sc_signal<bool>& CONTROL,
	 signal_std_logic& TS_OUT)
    : in(IN_), control(CONTROL), ts_out(TS_OUT)
  {
    SC_METHOD( entry );
    sensitive << in;
    sensitive << control;
  }

  // Process functionality in member function below
  void entry();
};


