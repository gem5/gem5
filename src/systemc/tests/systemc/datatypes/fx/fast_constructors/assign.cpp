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

  assign.cpp -- 

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
// assign.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Wed Dec 30 09:58:11 1998
// Status          : none
// 

#include <limits.h>

#define SC_INCLUDE_FX
#include "systemc.h"
#include "fx_precision_double.h"
 
typedef unsigned int   uint;
typedef unsigned short ushort;
typedef unsigned long  ulong;

#define SHOW_ASSIGN(a) cerr << #a << " : " << double(a) << " : " << a.to_string(SC_HEX) << "\n"
#define IDENT_ASSIGN(a) cerr << "--assign-Inf-Inf-Inf-Inf-Inf- " << a << "\n"

//----------------------------------------------------------------
static void test_fx_float_int()
{
  IDENT_ASSIGN("test_fx_float_int");

  sc_fxval a(0);
  sc_fxval b;
  sc_fxval c = b = -1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_float_uint()
{
  IDENT_ASSIGN("test_fx_float_uint");

  sc_fxval a(0u);
  sc_fxval b;
  sc_fxval c = b = (uint)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_float_short()
{
  IDENT_ASSIGN("test_fx_float_short");

  sc_fxval a((short)0);
  sc_fxval b;
  sc_fxval c = b = (short)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_float_ushort()
{
  IDENT_ASSIGN("test_fx_float_ushort");

  sc_fxval a((ushort)0);
  sc_fxval b;
  sc_fxval c = b = (ushort)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_float_long()
{
  IDENT_ASSIGN("test_fx_float_long");

  sc_fxval a(0L);
  sc_fxval b;
  sc_fxval c = b = -1L;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_float_ulong()
{
  IDENT_ASSIGN("test_fx_float_ulong");
  sc_fxval a(0UL);
  sc_fxval b;
  sc_fxval c = b = -1UL;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_float_float()
{
  IDENT_ASSIGN("test_fx_float_float");
  
  sc_fxval a(0.0f);
  sc_fxval b;
  sc_fxval c = b = -1.0f;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_float_double()
{
  IDENT_ASSIGN("test_fx_float_double");
  
  sc_fxval a(0.0);
  sc_fxval b;
  sc_fxval c = b = -1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

//----------------------------------------------------------------
static void test_fx_ufix_int()
{
  IDENT_ASSIGN("test_fx_ufix_int");

  sc_ufix a = 0;
  sc_ufix b;
  sc_ufix c = b = -1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_ufix_uint()
{
  IDENT_ASSIGN("test_fx_ufix_uint");

  sc_ufix a = (uint)0;
  sc_ufix b;
  sc_ufix c = b = (uint)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufix_short()
{
  IDENT_ASSIGN("test_fx_ufix_short");

  sc_ufix a = (short)0;
  sc_ufix b;
  sc_ufix c = b = (short)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_ufix_ushort()
{
  IDENT_ASSIGN("test_fx_ufix_ushort");

  sc_ufix a = (ushort)0;
  sc_ufix b;
  sc_ufix c = b = (ushort)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufix_long()
{
  IDENT_ASSIGN("test_fx_ufix_long");

  sc_ufix a = (long)0;
  sc_ufix b;
  sc_ufix c = b = (long)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufix_ulong()
{
  IDENT_ASSIGN("test_fx_ufix_ulong");
  sc_ufix a = (ulong)0;
  sc_ufix b;
  sc_ufix c = b = (ulong)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufix_float()
{
  IDENT_ASSIGN("test_fx_ufix_float");
  
  sc_ufix a = 0.0;
  sc_ufix b;
  sc_ufix c = b = -1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufix_double()
{
  IDENT_ASSIGN("test_fx_ufix_double");
  
  sc_ufix a = (double)0.0;
  sc_ufix b;
  sc_ufix c = b = (double)-1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

//----------------------------------------------------------------
static void test_fx_fix_int()
{
  IDENT_ASSIGN("test_fx_fix_int");

  sc_fix a = 0;
  sc_fix b;
  sc_fix c = b = -1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_fix_uint()
{
  IDENT_ASSIGN("test_fx_fix_uint");

  sc_fix a = (uint)0;
  sc_fix b;
  sc_fix c = b = (uint)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fix_short()
{
  IDENT_ASSIGN("test_fx_fix_short");

  sc_fix a = (short)0;
  sc_fix b;
  sc_fix c = b = (short)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_fix_ushort()
{
  IDENT_ASSIGN("test_fx_fix_ushort");

  sc_fix a = (ushort)0;
  sc_fix b;
  sc_fix c = b = (ushort)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fix_long()
{
  IDENT_ASSIGN("test_fx_fix_long");

  sc_fix a = (long)0;
  sc_fix b;
  sc_fix c = b = (long)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fix_ulong()
{
  IDENT_ASSIGN("test_fx_fix_ulong");
  sc_fix a = (ulong)0;
  sc_fix b;
  sc_fix c = b = (ulong)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fix_float()
{
  IDENT_ASSIGN("test_fx_fix_float");
  
  sc_fix a = 0.0;
  sc_fix b;
  sc_fix c = b = -1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fix_double()
{
  IDENT_ASSIGN("test_fx_fix_double");
  
  sc_fix a = (double)0.0;
  sc_fix b;
  sc_fix c = b = (double)-1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

//----------------------------------------------------------------
static void test_fx_fixed_int()
{
  IDENT_ASSIGN("test_fx_fixed_int");

  sc_fixed<8, 5> a = 0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = -1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_fixed_uint()
{
  IDENT_ASSIGN("test_fx_fixed_uint");

  sc_fixed<8, 5> a = (uint)0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (uint)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fixed_short()
{
  IDENT_ASSIGN("test_fx_fixed_short");

  sc_fixed<8, 5> a = (short)0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (short)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_fixed_ushort()
{
  IDENT_ASSIGN("test_fx_fixed_ushort");

  sc_fixed<8, 5> a = (ushort)0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (ushort)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fixed_long()
{
  IDENT_ASSIGN("test_fx_fixed_long");

  sc_fixed<8, 5> a = (long)0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (long)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fixed_ulong()
{
  IDENT_ASSIGN("test_fx_fixed_ulong");
  sc_fixed<8, 5> a = (ulong)0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (ulong)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fixed_float()
{
  IDENT_ASSIGN("test_fx_fixed_float");
  
  sc_fixed<8, 5> a = 0.0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = -1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_fixed_double()
{
  IDENT_ASSIGN("test_fx_fixed_double");
  
  sc_fixed<8, 5> a = (double)0.0;
  sc_fixed<8, 5> b;
  sc_fixed<8, 5> c = b = (double)-1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

//----------------------------------------------------------------
static void test_fx_ufixed_int()
{
  IDENT_ASSIGN("test_fx_ufixed_int");

  sc_ufixed<8, 5> a = 0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = -1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_ufixed_uint()
{
  IDENT_ASSIGN("test_fx_ufixed_uint");

  sc_ufixed<8, 5> a = (uint)0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (uint)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufixed_short()
{
  IDENT_ASSIGN("test_fx_ufixed_short");

  sc_ufixed<8, 5> a = (short)0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (short)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c);
}

static void test_fx_ufixed_ushort()
{
  IDENT_ASSIGN("test_fx_ufixed_ushort");

  sc_ufixed<8, 5> a = (ushort)0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (ushort)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufixed_long()
{
  IDENT_ASSIGN("test_fx_ufixed_long");

  sc_ufixed<8, 5> a = (long)0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (long)-1;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufixed_ulong()
{
  IDENT_ASSIGN("test_fx_ufixed_ulong");
  sc_ufixed<8, 5> a = (ulong)0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (ulong)abs(-1);

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufixed_float()
{
  IDENT_ASSIGN("test_fx_ufixed_float");
  
  sc_ufixed<8, 5> a = 0.0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = -1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

static void test_fx_ufixed_double()
{
  IDENT_ASSIGN("test_fx_ufixed_double");
  
  sc_ufixed<8, 5> a = (double)0.0;
  sc_ufixed<8, 5> b;
  sc_ufixed<8, 5> c = b = (double)-1.0;

  SHOW_ASSIGN(a); SHOW_ASSIGN(b); SHOW_ASSIGN(c); 
}

void assign()
{
  cerr << "************** assign test_fx_float_\n";
  test_fx_float_int();
  test_fx_float_uint();
  test_fx_float_short();
  test_fx_float_ushort();
  test_fx_float_long();
  test_fx_float_ulong();
  test_fx_float_float();
  test_fx_float_double();
  cerr << "************** assign test_fx_ufix_\n";
  test_fx_ufix_int();
  test_fx_ufix_uint();
  test_fx_ufix_short();
  test_fx_ufix_ushort();
  test_fx_ufix_long();
  test_fx_ufix_ulong();
  test_fx_ufix_float();
  test_fx_ufix_double();
  cerr << "************** assign test_fx_fix_\n";
  test_fx_fix_int();
  test_fx_fix_uint();
  test_fx_fix_short();
  test_fx_fix_ushort();
  test_fx_fix_long();
  test_fx_fix_ulong();
  test_fx_fix_float();
  test_fx_fix_double();
  cerr << "************** assign test_fx_fixed_\n";
  test_fx_fixed_int();
  test_fx_fixed_uint();
  test_fx_fixed_short();
  test_fx_fixed_ushort();
  test_fx_fixed_long();
  test_fx_fixed_ulong();
  test_fx_fixed_float();
  test_fx_fixed_double();
  cerr << "************** assign test_fx_ufixed_\n";
  test_fx_ufixed_int();
  test_fx_ufixed_uint();
  test_fx_ufixed_short();
  test_fx_ufixed_ushort();
  test_fx_ufixed_long();
  test_fx_ufixed_ulong();
  test_fx_ufixed_float();
  test_fx_ufixed_double();
}
