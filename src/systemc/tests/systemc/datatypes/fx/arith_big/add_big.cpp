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

  add_big.cpp -- 

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
// add_big.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Fri Jan 15 12:35:43 1999
// Status          : none
// 


#define SC_INCLUDE_FX
#include "systemc.h"


#define SHOW(r, m) \
out << r.to_string(SC_BIN) << " " << flush; \
out << r.to_string(SC_HEX) << " " << flush; \
out << m.to_string(SC_BIN) << " " << flush; \
out << m.to_string(SC_HEX) << "\n" << flush


#define BIG_ARROW(T_op)							      \
{									      \
  out << "______________________ " #T_op " ______________________\n";         \
  out << "______________________ UP        ______________________\n";         \
  sc_fxtype_params fooc(T_WL, T_IWL, SC_RND, SC_SAT);                        \
  T_op m(1); \
  T_op r(1); \
  int i;								      \
  for (i = 0; i < T_WL + 10; ++i)						      \
    {									      \
      r += r + (r >> 1);								      \
      SHOW(r, m);							      \
      m *= 16;								      \
    }									      \
  out << "______________________ DOWN      ______________________\n";         \
  m = 1;								      \
  for (i = 0; i < T_WL + 10; ++i)			                      \
    {									      \
      r -= m;								      \
      SHOW(r, m);							      \
      m *= 16;								      \
    }									      \
}

#define T_FX_FLOAT  sc_fxval
#define T_FX_UFIX   sc_ufix
#define T_FX_FIX    sc_fix
#define T_FX_FIXED  sc_fixed<T_WL, T_IWL>
#define T_FX_UFIXED sc_ufixed<T_WL, T_IWL>

#define THE_BIG_ARROW							      \
BIG_ARROW(T_FX_FLOAT)							      \
BIG_ARROW(T_FX_UFIX)							      \
BIG_ARROW(T_FX_FIX)							      \
BIG_ARROW(T_FX_FIXED)							      \
BIG_ARROW(T_FX_UFIXED)


void add_big(ostream& out)
{
  out.precision(15);

#define T_WL 16
#define T_IWL T_WL
  out << "************* add_big " << T_WL << " ***************\n";
  THE_BIG_ARROW;
#undef T_WL
#undef T_IWL
#define T_WL 67
#define T_IWL T_WL
  out << "************* add_big " << T_WL << " ***************\n";
  THE_BIG_ARROW;
#undef T_WL
#undef T_IWL
#define T_WL 150
#define T_IWL T_WL
  out << "************* add_big " << T_WL << " ***************\n";
  THE_BIG_ARROW;
}
