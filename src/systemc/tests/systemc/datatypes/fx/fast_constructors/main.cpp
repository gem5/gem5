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

  main.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// This may look like C code, but it is really -*- C++ -*-
// 
// 4.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Wed Dec 30 08:50:50 1998
// Status          : none
// 

#include <limits.h>

#define SC_INCLUDE_FX
#include "systemc.h"
#include "fx_precision_double.h"

typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned long  ulong;
 
extern void default_constructor();
extern void assign_constructor();
extern void default_assign();
extern void assign();
extern void array();

int sc_main( int, char** )
{
  cerr.precision(15);
  default_constructor();
  assign_constructor();
  default_assign();
  assign();

  array();

  return 0;
}
