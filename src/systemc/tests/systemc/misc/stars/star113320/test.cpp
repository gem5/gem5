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

  test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*
Ulli Holtmann, Nov/10/00

I would like to convert between sc_int and sc_bv like in this example:
*/

#include "systemc.h"

int sc_main(int argc, char* arg[]) 
{
    sc_int<10> a;
    sc_bv<10> b;
    sc_int<7> c;

    a=10; cout << "a="<< a << "\n";
    b=a; cout << "b="<< b << "\n";
    c=b; cout << "c="<< c << "\n";

    return 0;
}

/*
This doesn't work on with either SC5.0 or g++ for different reasons. 

SC 5.0
------------------------------------
compile error at assignment "c=b" == sc_int<7> = sc_bv<10> :

"/view/ccss_ulli/vobs/abc/src/scenic/scenery/include/sc_lv.h", line 165: Error: The function "convert" must have a prototype.
"str.cc", line 12:     Where: While instantiating "sc_bv_ns::sc_lv<7>::sc_lv(const sc_bv_ns::sc_bv_base&)".
"str.cc", line 12:     Where: Instantiated from non-template code.

file sc_lv.h:
#if !defined(__SUNPRO_CC)
  template<class Y> sc_lv(const sc_proxy<Y>& y)
    {init(y.back_cast().length()); assign_(y);}
  template<class T> sc_lv& operator=(const sc_proxy<T>& t)
    { sc_bv_ns::assign_(*this,t); return *this;}
#else
  // at least some constructors
  sc_lv(const sc_bv<W>& t){convert(t);verify_length();}
  sc_lv(const sc_lv_base& y):sc_lv_base(y){verify_length();}
  sc_lv(const sc_bv_base& t){convert(t);verify_length();}
#endif

Apparently, the "convert" function is undeclared.


With gcc
------------------------------------
compiles fine, but then Abort at assignment "b=a" == sc_bv<10> = sc_int<10> :

a=10
Abort (core dumped)



I am using SystemC 1.0.1 (stellar does not yet have a 1.0.1 release entry). on 
a Solaris 5.5.1 host with SC5.0 or g++ 2.95.2
*/

