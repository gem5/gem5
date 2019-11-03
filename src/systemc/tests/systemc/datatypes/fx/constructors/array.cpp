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

  array.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// array.cxx -- 
// Copyright Synopsys 1998
// Author          : Ric Hilderink
// Created On      : Wed Dec 30 11:55:50 1998
// Status          : none
// 

#include <limits.h>
#define SC_INCLUDE_FX
#include "systemc.h"

// Without this, conversions use the implicit
// operator double, which is both lossy and
// can cause UB in certain cases.
template <typename From>
class converter {
  From val;
public:
  converter(From val): val(val) {}
  operator int() const            { return val.to_int();    }
  operator unsigned int() const   { return val.to_uint();   }
  operator short() const          { return val.to_short();  }
  operator unsigned short() const { return val.to_ushort(); }
  operator long() const           { return val.to_long();   }
  operator unsigned long() const  { return val.to_ulong();  }
  operator float() const          { return val.to_float();  }
  operator double() const         { return val.to_double(); }
};

template <typename To, typename From>
To convert(From val) {
  return converter<From>(val);
}


template <typename T>
void show(char const *const name, int i, T val) {
  cerr << name << "[" << i << "] : " << val.to_double() << " : " << val.to_string(SC_HEX) << "\n";
}

template <typename T, typename U>
void test_fx(U a_mul, U b_init, U b_mul, char const *t_name, char const *u_name)
{
  cerr << "--array-Inf-Inf-Inf-Inf-Inf- test_fx_" << t_name << "_" << u_name << "\n";

  T a(static_cast<U>(0));
  T b(b_init);
  show("a", 0, a);
  show("b", 0, b);

  for (U i = 1; i < 4; ++i) {
    // Be careful here. We don't want a T to ever be implicitly converted,
    // else we have a double which cannot be safely cast, but converting early
    // means we can be subject to implicit upcasts. That's minimized by having all
    // of the values of type U, but if U is smaller than int it can still be
    // implicitly upcast, ergo the outer cast.
    a = static_cast<U>(i * i * a_mul);
    b = static_cast<U>(convert<U>(b) * i * b_mul);
    show("a", i, a);
    show("b", i, b);
  }
}

template <typename T>
void batch_test_fx(char const *t_name, int uinit) {
  cerr << "************** array test_fx_" << t_name << "_\n";
  test_fx<T, int           >(1,     -1, -1, t_name, "int");
  test_fx<T, unsigned int  >(1,  uinit, -1, t_name, "uint");
  test_fx<T, short         >(1,     -1, -1, t_name, "short");
  test_fx<T, unsigned short>(1,  uinit, -1, t_name, "ushort");
  test_fx<T, long          >(1,     -1, -1, t_name, "long");
  test_fx<T, unsigned long >(1,  uinit, -1, t_name, "ulong");
  test_fx<T, float >(1.123456789f, -1.987654321f, -1.789654123f, t_name, "float");
  test_fx<T, double>(1.123456789,  -1.987654321,  -1.789654123,  t_name, "double");
}

void array() {
  batch_test_fx<sc_fxval>("float", -1);
  batch_test_fx<sc_ufix>("ufix", -1);
  batch_test_fx<sc_fix>("fix", -1);
  batch_test_fx<sc_fixed<8, 5> >("fixed", 1);
  batch_test_fx<sc_ufixed<8, 5> >("ufixed", 1);
}
