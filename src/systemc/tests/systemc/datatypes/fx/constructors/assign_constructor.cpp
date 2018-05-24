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

  assign_constructor.cpp -- 

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
// assign_constructor.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Wed Dec 30 09:47:21 1998
// Status          : none
// 

#include <limits.h>
#include <float.h>
#define SC_INCLUDE_FX
#define SC_FXVAL_IMPLICIT_CONV
#include "systemc.h"
 
typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned long  ulong;

#define SHOW_ASSIGN_CONSTRUCTORS(a) cerr << #a << " : " << double(a) << " : " << a.to_string(SC_HEX) << "\n"
#define IDENT_ASSIGN_CONSTRUCTORS(a) cerr << "--assign_constructor-Inf-Inf-Inf-Inf-Inf- " << a << "\n"

//----------------------------------------------------------------
// copy-initialisation from builtin types no longer works with sc_fxval, 
// unless SC_FXVAL_IMPLICIT_CONV is defined

#if defined(SC_FXVAL_IMPLICIT_CONV)
static void test_fx_float_int()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_int");

  sc_fxval a = 0;
  sc_fxval b = 1;
  sc_fxval c = -1;
  sc_fxval d = INT_MAX;
  sc_fxval e = INT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_uint()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_uint");

  sc_fxval a = (uint)0;
  sc_fxval b = (uint)1;
  sc_fxval c = (uint)-1;
  sc_fxval d = UINT_MAX;
  sc_fxval e = (uint)abs(INT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_short()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_short");

  sc_fxval a = (short)0;
  sc_fxval b = (short)1;
  sc_fxval c = (short)-1;
  sc_fxval d = SHRT_MAX;
  sc_fxval e = SHRT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_ushort()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_ushort");

  sc_fxval a = (ushort)0;
  sc_fxval b = (ushort)1;
  sc_fxval c = (ushort)-1;
  sc_fxval d = USHRT_MAX;
  sc_fxval e = (ushort)abs(SHRT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_long()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_long");

  sc_fxval a = (long)0;
  sc_fxval b = (long)1;
  sc_fxval c = (long)-1;
  sc_fxval d = LONG_MAX;
  sc_fxval e = LONG_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_ulong()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_ulong");

  sc_fxval a = (ulong)0;
  sc_fxval b = (ulong)1;
  sc_fxval c = (ulong)-1;
  sc_fxval d = ULONG_MAX;
  sc_fxval e = (ulong)abs(LONG_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_float()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_float");
  
  sc_fxval a = 0.0;
  sc_fxval b = 1.0;
  sc_fxval c = -1.0;
  sc_fxval d = FLT_MAX;
  sc_fxval e = FLT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_float_double()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_float_double");
  
  sc_fxval a = (double)0.0;
  sc_fxval b = (double)1.0;
  sc_fxval c = (double)-1.0;
  sc_fxval d = DBL_MAX;
  sc_fxval e = DBL_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}
#endif // defined(SC_FXVAL_IMPLICIT_CONV)

//----------------------------------------------------------------
static void test_fx_ufix_int()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_int");

  sc_ufix a = 0;
  sc_ufix b = 1;
  sc_ufix c = -1;
  sc_ufix d = INT_MAX;
  sc_ufix e = INT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_uint()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_uint");

  sc_ufix a = (uint)0;
  sc_ufix b = (uint)1;
  sc_ufix c = (uint)-1;
  sc_ufix d = UINT_MAX;
  sc_ufix e = (uint)abs(INT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_short()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_short");

  sc_ufix a = (short)0;
  sc_ufix b = (short)1;
  sc_ufix c = (short)-1;
  sc_ufix d = SHRT_MAX;
  sc_ufix e = SHRT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_ushort()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_ushort");

  sc_ufix a = (ushort)0;
  sc_ufix b = (ushort)1;
  sc_ufix c = (ushort)-1;
  sc_ufix d = USHRT_MAX;
  sc_ufix e = (ushort)abs(SHRT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_long()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_long");

  sc_ufix a = (long)0;
  sc_ufix b = (long)1;
  sc_ufix c = (long)-1;
  sc_ufix d = LONG_MAX;
  sc_ufix e = LONG_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_ulong()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_ulong");

  sc_ufix a = (ulong)0;
  sc_ufix b = (ulong)1;
  sc_ufix c = (ulong)-1;
  sc_ufix d = ULONG_MAX;
  sc_ufix e = (ulong)abs(LONG_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_float()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_float");
  
  sc_ufix a = 0.0;
  sc_ufix b = 1.0;
  sc_ufix c = -1.0;
   sc_ufix d = FLT_MAX;
   sc_ufix e = FLT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufix_double()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufix_double");
  
  sc_ufix a = (double)0.0;
  sc_ufix b = (double)1.0;
  sc_ufix c = (double)-1.0;
   sc_ufix d = DBL_MAX;
   sc_ufix e = DBL_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

//----------------------------------------------------------------
static void test_fx_fix_int()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_int");

  sc_fix a = 0;
  sc_fix b = 1;
  sc_fix c = -1;
  sc_fix d = INT_MAX;
  sc_fix e = INT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_uint()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_uint");

  sc_fix a = (uint)0;
  sc_fix b = (uint)1;
  sc_fix c = (uint)-1;
  sc_fix d = UINT_MAX;
  sc_fix e = (uint)abs(INT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_short()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_short");

  sc_fix a = (short)0;
  sc_fix b = (short)1;
  sc_fix c = (short)-1;
  sc_fix d = SHRT_MAX;
  sc_fix e = SHRT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_ushort()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_ushort");

  sc_fix a = (ushort)0;
  sc_fix b = (ushort)1;
  sc_fix c = (ushort)-1;
  sc_fix d = USHRT_MAX;
  sc_fix e = (ushort)abs(SHRT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_long()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_long");

  sc_fix a = (long)0;
  sc_fix b = (long)1;
  sc_fix c = (long)-1;
  sc_fix d = LONG_MAX;
  sc_fix e = LONG_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_ulong()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_ulong");

  sc_fix a = (ulong)0;
  sc_fix b = (ulong)1;
  sc_fix c = (ulong)-1;
  sc_fix d = ULONG_MAX;
  sc_fix e = (ulong)abs(LONG_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_float()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_float");
  
  sc_fix a = 0.0;
  sc_fix b = 1.0;
  sc_fix c = -1.0;
   sc_fix d = FLT_MAX;
   sc_fix e = FLT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fix_double()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fix_double");
  
  sc_fix a = (double)0.0;
  sc_fix b = (double)1.0;
  sc_fix c = (double)-1.0;
   sc_fix d = DBL_MAX;
   sc_fix e = DBL_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

//----------------------------------------------------------------
static void test_fx_fixed_int()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_int");

  sc_fixed<8, 5> a = 0;
  sc_fixed<8, 5> b = 1;
  sc_fixed<8, 5> c = -1;
   sc_fixed<8, 5> d = INT_MAX;
   sc_fixed<8, 5> e = INT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_uint()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_uint");

  sc_fixed<8, 5> a = (uint)0;
  sc_fixed<8, 5> b = (uint)1;
   sc_fixed<8, 5> c = (uint)-1;
   sc_fixed<8, 5> d = UINT_MAX;
   sc_fixed<8, 5> e = (uint)abs(INT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_short()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_short");

  sc_fixed<8, 5> a = (short)0;
  sc_fixed<8, 5> b = (short)1;
  sc_fixed<8, 5> c = (short)-1;
   sc_fixed<8, 5> d = SHRT_MAX;
   sc_fixed<8, 5> e = SHRT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_ushort()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_ushort");

  sc_fixed<8, 5> a = (ushort)0;
  sc_fixed<8, 5> b = (ushort)1;
   sc_fixed<8, 5> c = (ushort)-1;
   sc_fixed<8, 5> d = USHRT_MAX;
   sc_fixed<8, 5> e = (ushort)abs(SHRT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_long()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_long");

  sc_fixed<8, 5> a = (long)0;
  sc_fixed<8, 5> b = (long)1;
  sc_fixed<8, 5> c = (long)-1;
   sc_fixed<8, 5> d = LONG_MAX;
   sc_fixed<8, 5> e = LONG_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_ulong()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_ulong");

  sc_fixed<8, 5> a = (ulong)0;
  sc_fixed<8, 5> b = (ulong)1;
   sc_fixed<8, 5> c = (ulong)-1;
   sc_fixed<8, 5> d = ULONG_MAX;
   sc_fixed<8, 5> e = (ulong)abs(LONG_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_float()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_float");
  
  sc_fixed<8, 5> a = 0.0;
  sc_fixed<8, 5> b = 1.0;
  sc_fixed<8, 5> c = -1.0;
   sc_fixed<8, 5> d = FLT_MAX;
   sc_fixed<8, 5> e = FLT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_fixed_double()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_fixed_double");
  
  sc_fixed<8, 5> a = (double)0.0;
  sc_fixed<8, 5> b = (double)1.0;
  sc_fixed<8, 5> c = (double)-1.0;
   sc_fixed<8, 5> d = DBL_MAX;
   sc_fixed<8, 5> e = DBL_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

//----------------------------------------------------------------
static void test_fx_ufixed_int()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_int");

  sc_ufixed<8, 5> a = 0;
  sc_ufixed<8, 5> b = 1;
  sc_ufixed<8, 5> c = -1;
   sc_ufixed<8, 5> d = INT_MAX;
   sc_ufixed<8, 5> e = INT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_uint()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_uint");

  sc_ufixed<8, 5> a = (uint)0;
  sc_ufixed<8, 5> b = (uint)1;
   sc_ufixed<8, 5> c = (uint)-1;
   sc_ufixed<8, 5> d = UINT_MAX;
   sc_ufixed<8, 5> e = (uint)abs(INT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_short()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_short");

  sc_ufixed<8, 5> a = (short)0;
  sc_ufixed<8, 5> b = (short)1;
  sc_ufixed<8, 5> c = (short)-1;
   sc_ufixed<8, 5> d = SHRT_MAX;
   sc_ufixed<8, 5> e = SHRT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_ushort()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_ushort");

  sc_ufixed<8, 5> a = (ushort)0;
  sc_ufixed<8, 5> b = (ushort)1;
   sc_ufixed<8, 5> c = (ushort)-1;
   sc_ufixed<8, 5> d = USHRT_MAX;
   sc_ufixed<8, 5> e = (ushort)abs(SHRT_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_long()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_long");

  sc_ufixed<8, 5> a = (long)0;
  sc_ufixed<8, 5> b = (long)1;
  sc_ufixed<8, 5> c = (long)-1;
   sc_ufixed<8, 5> d = LONG_MAX;
   sc_ufixed<8, 5> e = LONG_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_ulong()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_ulong");

  sc_ufixed<8, 5> a = (ulong)0;
  sc_ufixed<8, 5> b = (ulong)1;
   sc_ufixed<8, 5> c = (ulong)-1;
   sc_ufixed<8, 5> d = ULONG_MAX;
   sc_ufixed<8, 5> e = (ulong)abs(LONG_MIN);

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b);  SHOW_ASSIGN_CONSTRUCTORS(c); SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_float()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_float");
  
  sc_ufixed<8, 5> a = 0.0;
  sc_ufixed<8, 5> b = 1.0;
  sc_ufixed<8, 5> c = -1.0;
   sc_ufixed<8, 5> d = FLT_MAX;
   sc_ufixed<8, 5> e = FLT_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

static void test_fx_ufixed_double()
{
  IDENT_ASSIGN_CONSTRUCTORS("test_fx_ufixed_double");
  
  sc_ufixed<8, 5> a = (double)0.0;
  sc_ufixed<8, 5> b = (double)1.0;
  sc_ufixed<8, 5> c = (double)-1.0;
   sc_ufixed<8, 5> d = DBL_MAX;
   sc_ufixed<8, 5> e = DBL_MIN;

  SHOW_ASSIGN_CONSTRUCTORS(a); SHOW_ASSIGN_CONSTRUCTORS(b); SHOW_ASSIGN_CONSTRUCTORS(c);  SHOW_ASSIGN_CONSTRUCTORS(d); SHOW_ASSIGN_CONSTRUCTORS(e);
}

void assign_constructor()
{
#if defined(SC_FXVAL_IMPLICIT_CONV)
  cerr << "************** assign_constructor test_fx_float_\n";
  test_fx_float_int();
  test_fx_float_uint();
  test_fx_float_short();
  test_fx_float_ushort();
  test_fx_float_long();
  test_fx_float_ulong();
  test_fx_float_float();
  test_fx_float_double();
#endif // defined(SC_FXVAL_IMPLICIT_CONV)
  cerr << "************** assign_constructor test_fx_ufix_\n";
  test_fx_ufix_int();
  test_fx_ufix_uint();
  test_fx_ufix_short();
  test_fx_ufix_ushort();
  test_fx_ufix_long();
  test_fx_ufix_ulong();
  test_fx_ufix_float();
  test_fx_ufix_double();
  cerr << "************** assign_constructor test_fx_fix_\n";
  test_fx_fix_int();
  test_fx_fix_uint();
  test_fx_fix_short();
  test_fx_fix_ushort();
  test_fx_fix_long();
  test_fx_fix_ulong();
  test_fx_fix_float();
  test_fx_fix_double();
  cerr << "************** assign_constructor test_fx_fixed_\n";
  test_fx_fixed_int();
  test_fx_fixed_uint();
  test_fx_fixed_short();
  test_fx_fixed_ushort();
  test_fx_fixed_long();
  test_fx_fixed_ulong();
  test_fx_fixed_float();
  test_fx_fixed_double();
  cerr << "************** assign_constructor test_fx_ufixed_\n";
  test_fx_ufixed_int();
  test_fx_ufixed_uint();
  test_fx_ufixed_short();
  test_fx_ufixed_ushort();
  test_fx_ufixed_long();
  test_fx_ufixed_ulong();
  test_fx_ufixed_float();
  test_fx_ufixed_double();
}
