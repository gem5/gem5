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

  pipeline.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename pipeline.cc */
/* This is the implementation file for module `pipeline' */

#include "systemc.h"
#include "f_stage1.h"
#include "f_stage2.h"
#include "stage1_2.h"
#include "f_stage3.h"
#include "pipeline.h"
#include "f_pipeline.h"

void  f_pipeline(const char *NAME,
		 sc_clock& CLK,
		 const sc_signal<double>& IN1,
		 const sc_signal<double>& IN2,
		 sc_signal<double>& OUT_)
{
  SC_NEW(pipeline(NAME, CLK, IN1, IN2, OUT_));
}
