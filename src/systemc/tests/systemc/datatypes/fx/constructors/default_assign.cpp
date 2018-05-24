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

  default_assign.cpp -- 

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
// default_assign.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Wed Dec 30 09:52:54 1998
// Status          : none
// 

#include <limits.h>
#include <float.h>
#define SC_INCLUDE_FX
#include "systemc.h"
 
typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned long  ulong;
 
#define SHOW_DEFAULT_ASSIGN(a) cerr << #a << " : " << double(a) << " : " << a.to_string(SC_HEX) << "\n"
#define IDENT_DEFAULT_ASSIGN(a) cerr << "--default_assign-Inf-Inf-Inf-Inf-Inf- " << a << "\n"

//-------------------------------------------------------
static void test_fx_float_int()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_int");

  sc_fxval a; a = 0;
  sc_fxval b; b = 1;
  sc_fxval c; c = -1;
  sc_fxval d; d = INT_MAX;
  sc_fxval e; e = INT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_uint()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_uint");

  sc_fxval a; a = (uint)0;
  sc_fxval b; b = (uint)1;
  sc_fxval c; c = (uint)-1;
  sc_fxval d; d = UINT_MAX;
  sc_fxval e; e = (uint)abs(INT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_short()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_short");

  sc_fxval a; a = (short)0;
  sc_fxval b; b = (short)1;
  sc_fxval c; c = (short)-1;
  sc_fxval d; d = SHRT_MAX;
  sc_fxval e; e = SHRT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_ushort()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_ushort");

  sc_fxval a; a = (ushort)0;
  sc_fxval b; b = (ushort)1;
  sc_fxval c; c = (ushort)-1;
  sc_fxval d; d = USHRT_MAX;
  sc_fxval e; e = (ushort)abs(SHRT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_long()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_long");

  sc_fxval a; a = (long)0;
  sc_fxval b; b = (long)1;
  sc_fxval c; c = (long)-1;
  sc_fxval d; d = LONG_MAX;
  sc_fxval e; e = LONG_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_ulong()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_ulong");

  sc_fxval a; a = (ulong)0;
  sc_fxval b; b = (ulong)1;
  sc_fxval c; c = (ulong)-1;
  sc_fxval d; d = ULONG_MAX;
  sc_fxval e; e = (ulong)abs(LONG_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_float()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_float");
  
  sc_fxval a; a = 0.0;
  sc_fxval b; b = 1.0;
  sc_fxval c; c = -1.0;
  sc_fxval d; d = FLT_MAX;
  sc_fxval e; e = FLT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_float_double()
{
  IDENT_DEFAULT_ASSIGN("test_fx_float_double");
  
  sc_fxval a; a = (double)0.0;
  sc_fxval b; b = (double)1.0;
  sc_fxval c; c = (double)-1.0;
  sc_fxval d; d = DBL_MAX;
  sc_fxval e; e = DBL_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

//-------------------------------------------------------
static void test_fx_ufix_int()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_int");

  sc_ufix a; a = 0;
  sc_ufix b; b = 1;
  sc_ufix c; c = -1;
  sc_ufix d; d = INT_MAX;
  sc_ufix e; e = INT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_uint()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_uint");

  sc_ufix a; a = (uint)0;
  sc_ufix b; b = (uint)1;
  sc_ufix c; c = (uint)-1;
  sc_ufix d; d = UINT_MAX;
  sc_ufix e; e = (uint)abs(INT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_short()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_short");

  sc_ufix a; a = (short)0;
  sc_ufix b; b = (short)1;
  sc_ufix c; c = (short)-1;
  sc_ufix d; d = SHRT_MAX;
  sc_ufix e; e = SHRT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_ushort()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_ushort");

  sc_ufix a; a = (ushort)0;
  sc_ufix b; b = (ushort)1;
  sc_ufix c; c = (ushort)-1;
  sc_ufix d; d = USHRT_MAX;
  sc_ufix e; e = (ushort)abs(SHRT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_long()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_long");

  sc_ufix a; a = (long)0;
  sc_ufix b; b = (long)1;
  sc_ufix c; c = (long)-1;
  sc_ufix d; d = LONG_MAX;
  sc_ufix e; e = LONG_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_ulong()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_ulong");

  sc_ufix a; a = (ulong)0;
  sc_ufix b; b = (ulong)1;
  sc_ufix c; c = (ulong)-1;
  sc_ufix d; d = ULONG_MAX;
  sc_ufix e; e = (ulong)abs(LONG_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_float()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_float");
  
  sc_ufix a; a = 0.0;
  sc_ufix b; b = 1.0;
  sc_ufix c; c = -1.0;
   sc_ufix d; d = FLT_MAX;
   sc_ufix e; e = FLT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufix_double()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufix_double");
  
  sc_ufix a; a = (double)0.0;
  sc_ufix b; b = (double)1.0;
  sc_ufix c; c = (double)-1.0;
   sc_ufix d; d = DBL_MAX;
   sc_ufix e; e = DBL_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

//-------------------------------------------------------
static void test_fx_fix_int()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_int");

  sc_fix a; a = 0;
  sc_fix b; b = 1;
  sc_fix c; c = -1;
  sc_fix d; d = INT_MAX;
  sc_fix e; e = INT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_uint()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_uint");

  sc_fix a; a = (uint)0;
  sc_fix b; b = (uint)1;
  sc_fix c; c = (uint)-1;
  sc_fix d; d = UINT_MAX;
  sc_fix e; e = (uint)abs(INT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_short()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_short");

  sc_fix a; a = (short)0;
  sc_fix b; b = (short)1;
  sc_fix c; c = (short)-1;
  sc_fix d; d = SHRT_MAX;
  sc_fix e; e = SHRT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_ushort()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_ushort");

  sc_fix a; a = (ushort)0;
  sc_fix b; b = (ushort)1;
  sc_fix c; c = (ushort)-1;
  sc_fix d; d = USHRT_MAX;
  sc_fix e; e = (ushort)abs(SHRT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_long()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_long");

  sc_fix a; a = (long)0;
  sc_fix b; b = (long)1;
  sc_fix c; c = (long)-1;
  sc_fix d; d = LONG_MAX;
  sc_fix e; e = LONG_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_ulong()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_ulong");

  sc_fix a; a = (ulong)0;
  sc_fix b; b = (ulong)1;
  sc_fix c; c = (ulong)-1;
  sc_fix d; d = ULONG_MAX;
  sc_fix e; e = (ulong)abs(LONG_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_float()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_float");
  
  sc_fix a; a = 0.0;
  sc_fix b; b = 1.0;
  sc_fix c; c = -1.0;
   sc_fix d; d = FLT_MAX;
   sc_fix e; e = FLT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fix_double()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fix_double");
  
  sc_fix a; a = (double)0.0;
  sc_fix b; b = (double)1.0;
  sc_fix c; c = (double)-1.0;
   sc_fix d; d = DBL_MAX;
   sc_fix e; e = DBL_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

//-------------------------------------------------------
static void test_fx_fixed_int()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_int");

  sc_fixed<8, 5> a; a = 0;
  sc_fixed<8, 5> b; b = 1;
  sc_fixed<8, 5> c; c = -1;
   sc_fixed<8, 5> d; d = INT_MAX;
   sc_fixed<8, 5> e; e = INT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_uint()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_uint");

  sc_fixed<8, 5> a; a = (uint)0;
  sc_fixed<8, 5> b; b = (uint)1;
   sc_fixed<8, 5> c; c = (uint)-1;
   sc_fixed<8, 5> d; d = UINT_MAX;
   sc_fixed<8, 5> e; e = (uint)abs(INT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_short()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_short");

  sc_fixed<8, 5> a; a = (short)0;
  sc_fixed<8, 5> b; b = (short)1;
  sc_fixed<8, 5> c; c = (short)-1;
   sc_fixed<8, 5> d; d = SHRT_MAX;
   sc_fixed<8, 5> e; e = SHRT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_ushort()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_ushort");

  sc_fixed<8, 5> a; a = (ushort)0;
  sc_fixed<8, 5> b; b = (ushort)1;
   sc_fixed<8, 5> c; c = (ushort)-1;
   sc_fixed<8, 5> d; d = USHRT_MAX;
   sc_fixed<8, 5> e; e = (ushort)abs(SHRT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_long()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_long");

  sc_fixed<8, 5> a; a = (long)0;
  sc_fixed<8, 5> b; b = (long)1;
  sc_fixed<8, 5> c; c = (long)-1;
   sc_fixed<8, 5> d; d = LONG_MAX;
   sc_fixed<8, 5> e; e = LONG_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_ulong()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_ulong");

  sc_fixed<8, 5> a; a = (ulong)0;
  sc_fixed<8, 5> b; b = (ulong)1;
   sc_fixed<8, 5> c; c = (ulong)-1;
   sc_fixed<8, 5> d; d = ULONG_MAX;
   sc_fixed<8, 5> e; e = (ulong)abs(LONG_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_float()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_float");
  
  sc_fixed<8, 5> a; a = 0.0;
  sc_fixed<8, 5> b; b = 1.0;
  sc_fixed<8, 5> c; c = -1.0;
   sc_fixed<8, 5> d; d = FLT_MAX;
   sc_fixed<8, 5> e; e = FLT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_fixed_double()
{
  IDENT_DEFAULT_ASSIGN("test_fx_fixed_double");
  
  sc_fixed<8, 5> a; a = (double)0.0;
  sc_fixed<8, 5> b; b = (double)1.0;
  sc_fixed<8, 5> c; c = (double)-1.0;
   sc_fixed<8, 5> d; d = DBL_MAX;
   sc_fixed<8, 5> e; e = DBL_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

//-------------------------------------------------------
static void test_fx_ufixed_int()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_int");

  sc_ufixed<8, 5> a; a = 0;
  sc_ufixed<8, 5> b; b = 1;
  sc_ufixed<8, 5> c; c = -1;
   sc_ufixed<8, 5> d; d = INT_MAX;
   sc_ufixed<8, 5> e; e = INT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_uint()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_uint");

  sc_ufixed<8, 5> a; a = (uint)0;
  sc_ufixed<8, 5> b; b = (uint)1;
   sc_ufixed<8, 5> c; c = (uint)-1;
   sc_ufixed<8, 5> d; d = UINT_MAX;
   sc_ufixed<8, 5> e; e = (uint)abs(INT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_short()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_short");

  sc_ufixed<8, 5> a; a = (short)0;
  sc_ufixed<8, 5> b; b = (short)1;
  sc_ufixed<8, 5> c; c = (short)-1;
   sc_ufixed<8, 5> d; d = SHRT_MAX;
   sc_ufixed<8, 5> e; e = SHRT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_ushort()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_ushort");

  sc_ufixed<8, 5> a; a = (ushort)0;
  sc_ufixed<8, 5> b; b = (ushort)1;
   sc_ufixed<8, 5> c; c = (ushort)-1;
   sc_ufixed<8, 5> d; d = USHRT_MAX;
   sc_ufixed<8, 5> e; e = (ushort)abs(SHRT_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_long()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_long");

  sc_ufixed<8, 5> a; a = (long)0;
  sc_ufixed<8, 5> b; b = (long)1;
  sc_ufixed<8, 5> c; c = (long)-1;
   sc_ufixed<8, 5> d; d = LONG_MAX;
   sc_ufixed<8, 5> e; e = LONG_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_ulong()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_ulong");

  sc_ufixed<8, 5> a; a = (ulong)0;
  sc_ufixed<8, 5> b; b = (ulong)1;
   sc_ufixed<8, 5> c; c = (ulong)-1;
   sc_ufixed<8, 5> d; d = ULONG_MAX;
   sc_ufixed<8, 5> e; e = (ulong)abs(LONG_MIN);

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b);  SHOW_DEFAULT_ASSIGN(c); SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_float()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_float");
  
  sc_ufixed<8, 5> a; a = 0.0;
  sc_ufixed<8, 5> b; b = 1.0;
  sc_ufixed<8, 5> c; c = -1.0;
   sc_ufixed<8, 5> d; d = FLT_MAX;
   sc_ufixed<8, 5> e; e = FLT_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

static void test_fx_ufixed_double()
{
  IDENT_DEFAULT_ASSIGN("test_fx_ufixed_double");
  
  sc_ufixed<8, 5> a; a = (double)0.0;
  sc_ufixed<8, 5> b; b = (double)1.0;
  sc_ufixed<8, 5> c; c = (double)-1.0;
   sc_ufixed<8, 5> d; d = DBL_MAX;
   sc_ufixed<8, 5> e; e = DBL_MIN;

  SHOW_DEFAULT_ASSIGN(a); SHOW_DEFAULT_ASSIGN(b); SHOW_DEFAULT_ASSIGN(c);  SHOW_DEFAULT_ASSIGN(d); SHOW_DEFAULT_ASSIGN(e);
}

void default_assign()
{
  cerr << "************** default_assign fx_float\n";
  test_fx_float_int();
  test_fx_float_uint();
  test_fx_float_short();
  test_fx_float_ushort();
  test_fx_float_long();
  test_fx_float_ulong();
  test_fx_float_float();
  test_fx_float_double();

  cerr << "************** default_assign fx_ufix\n";
  test_fx_ufix_int();
  test_fx_ufix_uint();
  test_fx_ufix_short();
  test_fx_ufix_ushort();
  test_fx_ufix_long();
  test_fx_ufix_ulong();
  test_fx_ufix_float();
  test_fx_ufix_double();

  cerr << "************** default_assign fx_fix\n";
  test_fx_fix_int();
  test_fx_fix_uint();
  test_fx_fix_short();
  test_fx_fix_ushort();
  test_fx_fix_long();
  test_fx_fix_ulong();
  test_fx_fix_float();
  test_fx_fix_double();

  cerr << "************** default_assign fx_fixed\n";
  test_fx_fixed_int();
  test_fx_fixed_uint();
  test_fx_fixed_short();
  test_fx_fixed_ushort();
  test_fx_fixed_long();
  test_fx_fixed_ulong();
  test_fx_fixed_float();
  test_fx_fixed_double();

  cerr << "************** default_assign fx_ufixed\n";
  test_fx_ufixed_int();
  test_fx_ufixed_uint();
  test_fx_ufixed_short();
  test_fx_ufixed_ushort();
  test_fx_ufixed_long();
  test_fx_ufixed_ulong();
  test_fx_ufixed_float();
  test_fx_ufixed_double();
}
