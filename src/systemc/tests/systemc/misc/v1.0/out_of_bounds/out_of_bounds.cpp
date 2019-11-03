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

  out_of_bounds.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

//----------------------------------------------------------
// test the out of range of sc_bv<> and sc_lv<>
//----------------------------------------------------------
#include "systemc.h"

template<class X>
void test(int W)
{
  X x(W);
  try  {    x.range(W+1,0); }
  catch(const sc_report& s)
  {
    cout<<s.what()<<"\n";
  }
  try  {    x.range(0,W+1); }
  catch(const sc_report& s)
  {
    cout<<s.what()<<"\n";
  }
  try  {    x.range(W-1,-1); }
  catch(const sc_report& s)
  {
    cout<<s.what()<<"\n";
  }
  try  {    x.range(-1,W-1); }
  catch(const sc_report& s)
  {
    cout<<s.what()<<"\n";
  }
  catch(...)
    { cout<<"couldn''t catch anything\n";}
}

int sc_main(int, char**)
{
  const unsigned N = 2000;

  test<sc_bv<N> >(N);
  test<sc_lv<N> >(N);

  return 0;
}
