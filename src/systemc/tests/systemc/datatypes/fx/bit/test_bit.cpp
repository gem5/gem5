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

  test_bit.cpp -- 

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
// test_bit.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Thu Jan 21 10:27:26 1999
// Status          : none
// 


#define SC_INCLUDE_FX
#include "systemc.h"
#include "test_all.hh"


void test_bit(ostream& out, int wl, int iwl)
{
#undef T_WL
#undef T_IWL
#define T_WL 13
#define T_IWL 13
  TEST_BIT;
#undef T_WL
#undef T_IWL
#define T_WL 13
#define T_IWL 0
  TEST_BIT;
#undef T_WL
#undef T_IWL
#define T_WL 5
#define T_IWL 13
  TEST_BIT;

#undef T_WL
#undef T_IWL
#define T_WL 65
#define T_IWL 65
  TEST_BIT;
#undef T_WL
#undef T_IWL
#define T_WL 65
#define T_IWL 33
  TEST_BIT;
#undef T_WL
#undef T_IWL
#define T_WL 65
#define T_IWL 111
  TEST_BIT;
}
