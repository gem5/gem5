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

  operator_shift_right.cpp -- 

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
// operator_shift_right.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Fri Jan 15 14:08:43 1999
// Status          : none
// 


#include <limits.h>
#define SC_INCLUDE_FX
#include "systemc.h"



#define T_FX_FLOAT  sc_fxval
#define T_FX_UFIX   sc_ufix
#define T_FX_FIX    sc_fix
#define T_FX_FIXED  sc_fixed<T_WL, T_IWL>
#define T_FX_UFIXED sc_ufixed<T_WL, T_IWL>

#define SHOW(i, a) \
  out << i << " : " << a.to_string(SC_BIN) << "\n";

#if 0

#define SHIFT_RIGHT_OP(T_op)						      \
{									      \
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);		      \
  									      \
  out << "------------------ " #T_op " ---- " << T_WL << ", " << T_IWL << " --------------\n"; \
  T_op r(0x8000);							      \
  T_op m(0x8000, FX_off);						      \
  T_op q(FX_off);							      \
  int i;								      \
  for (i = 0; i < T_WL+10; ++i)						      \
    {									      \
      r = r >> 1;							      \
      r += 1;								      \
      q = m << i;							      \
      q += 1; 								      \
      SHOW(i, r);							      \
      SHOW(i, q);							      \
    }									      \
    r = r >> 0;								      \
    SHOW(0, r);								      \
    r = r >> -4;							      \
    SHOW(-4, r);							      \
    r >>= 0;								      \
    SHOW(0, r);								      \
    r >>= -4;							              \
    SHOW(-4, r);							      \
    q = m >> 0;								      \
    SHOW(0, m);								      \
    q = m >> -5;							      \
    SHOW(-5, q);							      \
}

#define SHIFT_RIGHT							      \
SHIFT_RIGHT_OP(T_FX_FLOAT)						      \
SHIFT_RIGHT_OP(T_FX_UFIX)						      \
SHIFT_RIGHT_OP(T_FX_FIX)						      \
SHIFT_RIGHT_OP(T_FX_FIXED)						      \
SHIFT_RIGHT_OP(T_FX_UFIXED)



void operator_shift_right(ostream& out)
{
#define T_WL 13
#define T_IWL 1
  SHIFT_RIGHT;
#undef T_WL
#undef T_IWL
#define T_WL 65
#define T_IWL 1
  SHIFT_RIGHT;
#undef T_WL
#undef T_IWL
#define T_WL 150
#define T_IWL 1
  SHIFT_RIGHT;
}

#else

#define QUOTE(x) #x

void operator_shift_right(ostream& out)
{
#define T_WL 13
#define T_IWL 1

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FLOAT) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FLOAT r(0x8000);
#include "fx_precision_double.h"
  T_FX_FLOAT m(0x8000);
  T_FX_FLOAT q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIX m(0x8000);
  T_FX_UFIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIX m(0x8000);
  T_FX_FIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIXED m(0x8000);
  T_FX_FIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIXED m(0x8000);
  T_FX_UFIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

#undef T_WL
#undef T_IWL
#define T_WL 65
#define T_IWL 1

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FLOAT) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FLOAT r(0x8000);
#include "fx_precision_double.h"
  T_FX_FLOAT m(0x8000);
  T_FX_FLOAT q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIX m(0x8000);
  T_FX_UFIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIX m(0x8000);
  T_FX_FIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIXED m(0x8000);
  T_FX_FIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIXED m(0x8000);
  T_FX_UFIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

#undef T_WL
#undef T_IWL
#define T_WL 150
#define T_IWL 1

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FLOAT) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FLOAT r(0x8000);
#include "fx_precision_double.h"
  T_FX_FLOAT m(0x8000);
  T_FX_FLOAT q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIX m(0x8000);
  T_FX_UFIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIX) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIX r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIX m(0x8000);
  T_FX_FIX q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_FIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_FIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_FIXED m(0x8000);
  T_FX_FIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}

{
  sc_fxtype_params fooc(T_WL, T_IWL, SC_TRN, SC_WRAP);

  out << "------------------ " QUOTE(T_FX_UFIXED) " ---- " << T_WL << ", "
     << T_IWL << " --------------\n";
  T_FX_UFIXED r(0x8000);
#include "fx_precision_double.h"
  T_FX_UFIXED m(0x8000);
  T_FX_UFIXED q;
#include "fx_precision_default.h"
  int i;
  for (i = 0; i < T_WL+10; ++i)
    {
      r = r >> 1;
      r += 1;
      q = m << i;
      q += 1;
      SHOW(i, r);
      SHOW(i, q);
    }
    r = r >> 0;
    SHOW(0, r);
    r = r >> -4;
    SHOW(-4, r);
    r >>= 0;
    SHOW(0, r);
    r >>= -4;
    SHOW(-4, r);
    q = m >> 0;
    SHOW(0, m);
    q = m >> -5;
    SHOW(-5, q);
}
}

#endif
