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

  range_fx.cpp -- 

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
// range_fx.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Mon Jan 11 13:03:27 1999
// Status          : none
// 


#include <limits.h>
#define SC_INCLUDE_FX
#define SC_FXVAL_IMPLICIT_FXVAL
#include "systemc.h"

#define T_FX_FLOAT  sc_fxval
#define T_FX_UFIX   sc_ufix
#define T_FX_FIX    sc_fix
#define T_FX_FIXED  sc_fixed<222,111>
#define T_FX_UFIXED sc_ufixed<222,111>

#define RANGE_TO_MIN(FX_TTT) \
{									      \
  FX_TTT x(1); \
  FX_TTT d(1); \
									      \
  int i;								      \
  for (i = 0; i < 300; ++i)						      \
    {									      \
      d = d / 2;							      \
      x += d;								      \
      out << i << " " << d.to_double() << " " << x.to_double() << " " << x.to_string(SC_BIN, SC_E) << "\n"; \
    }									      \
}									      


#define RANGE_MIN_MAX(FX_TTT) \
{									      \
  FX_TTT x(1); \
  FX_TTT d(4); \
  FX_TTT e(0.125);                                                           \
  int i;								      \
  for (i = 0; i < 300; ++i)						      \
    {									      \
      d = d * 2;							      \
      e = e / 2;                                                              \
      x += d + e;							      \
      out << i << " " << d.to_double() << " " << x.to_double() << " " << x.to_string(SC_BIN, SC_E) << "\n"; \
    }									      \
  for (i = 0; i < 300; ++i)						      \
    {									      \
      x -= (d + e);							      \
      out << i << " " << d.to_double() << " " << x.to_double() << " " << x.to_string(SC_BIN, SC_E) << "\n"; \
      d = d / 2;							      \
      e = e * 2;                                                              \
    }									      \
}

#define RANGE_TO_MAX(FX_TTT) \
{									      \
  FX_TTT x(1); \
  FX_TTT d(1); \
  int i;								      \
  for (i = 0; i < 300; ++i)						      \
    {									      \
      d = d * 2;							      \
      x += d;								      \
      out << i << " " << d.to_double() << " " << x.to_double() << " " << x.to_string(SC_BIN, SC_E) << "\n"; \
    }									      \
  for (i = 0; i < 300; ++i)						      \
    {									      \
      x -= d;								      \
      out << i << " " << d.to_double() << " " << x.to_double() << " " << x.to_string(SC_BIN, SC_E) << "\n"; \
      d = d / 2;							      \
    }									      \
}


static void range_to_min(ostream& out)
{
  RANGE_TO_MIN(T_FX_FLOAT);
  RANGE_TO_MIN(T_FX_UFIX);
  RANGE_TO_MIN(T_FX_FIX);
  RANGE_TO_MIN(T_FX_FIXED);
  RANGE_TO_MIN(T_FX_UFIXED);
}

static void range_to_max(ostream& out)
{
  RANGE_TO_MAX(T_FX_FLOAT);
  RANGE_TO_MAX(T_FX_UFIX);
  RANGE_TO_MAX(T_FX_FIX);
  RANGE_TO_MAX(T_FX_FIXED);
  RANGE_TO_MAX(T_FX_UFIXED);
}

static void range_min_max(ostream& out)
{
  RANGE_MIN_MAX(T_FX_FLOAT);
  RANGE_MIN_MAX(T_FX_UFIX);
  RANGE_MIN_MAX(T_FX_FIX);
  RANGE_MIN_MAX(T_FX_FIXED);
  RANGE_MIN_MAX(T_FX_UFIXED);
}


void range_fx(ostream& out)
{
  sc_fxtype_params fooCast(222, 111, SC_RND, SC_SAT);
  out << "************** range_FX_TTT\n";
  range_to_min(out);
  range_to_max(out);
  range_min_max(out);
}

