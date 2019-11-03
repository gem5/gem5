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

  blv.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

//----------------------------------------------------------
// test the basic functionality of sc_bv<> and sc_lv<>
//----------------------------------------------------------
#include "systemc.h"
#include <time.h>
#include "isaac.h"

QTIsaac<8> rng;

template<class T>
void compare(int W)
{ // functionality verification
  sc_bv_base x(W);
  T st;
  // initialize
  for(int i=0; i<W; i++)
  {
    bool la = (rng.rand()&1) == 0;
    x[i] = la;
    st[i] = la;
  }
  if(st.to_string()!=x.to_string())
    cout<<"\nERROR: x= "<<x<<" st= "<<st<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;
  if( (int) st.xor_reduce()!= x.xor_reduce())
    cout<<"\nERROR: st.xor_reduce="<<(short)st.xor_reduce()<<"; x.xor_reduce="<<
    (short)x.xor_reduce()<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;
  if( (int) x.or_reduce()!=st.or_reduce())
     cout<<"\nERROR: st.or_reduce="<<(short)st.or_reduce()<<
     "; x.or_reduce="<<(short)x.or_reduce()<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;
  if( (int) x.and_reduce()!=st.and_reduce())
    cout<<"\nERROR: st.and_reduce="<<(short)st.and_reduce()<<
    "; x.and_reduce="<<(short)x.and_reduce()<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;

  if((st,st).to_string()!=(x,x).to_string())
    cout<<"\nERROR: st,st="<<(st,st)<<
    "; x,x="<<(x,x)<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;
  int first = (int) ( (double) W * ((double)rng.rand() / (double)0x7fffffff));
  int second = (int) ( (double) W * ((double)rng.rand() / (double)0x7fffffff));
  if(st.range(first,second).to_string()!=x.range(first,second).to_string())
    cout<<"st.range("<<first<<","<<second<<")="<<st.range(first,second)<<
    "; x.range("<<first<<","<<second<<")="<<x.range(first,second)<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;

  sc_bv_base bv(2*W);
  sc_bv_base xv(2*W);
  bv = (st,st);
  xv = (x,x);
  if(bv.to_string()!=xv.to_string())
    cout<<"\nERROR: bv(st,st)="<<bv<<"; xv(x,x)="<<xv<<"\n"<<
    "st="<<st<<"\nx="<<x<< endl;
  int Len=0;
  if(first>second)
    Len = first-second;
  else
    Len = second-first;
  sc_bv_base br(Len+1);
  sc_bv_base xr(Len+1);
  br = st.range(first,second);
  xr = x.range(first,second);
  if(br.to_string()!=xr.to_string())
  {
    cout<<"\nERROR: br("<<first<<","<<second<<")!= xr("<<first<<","<<second
        <<")" << endl;
    br = st.range(first,second);
    xr = x.range(first,second);
    cout<<"br="<<br.to_string()<<"  xr="<<xr.to_string()<<
    "st.range="<<st.range(first,second)<<"  x.range="<<x.range(first,second)<<
    "st="<<st<<"\nx="<<x<< endl;
  }

  // verify assignments
  long ra = rng.rand();
  x  = ra;
  st = ra;
  if(st.to_string()!=x.to_string())
    cout<<"\nERROR (assignment): x= "<<x<<" st= "<<st<<" original long="<<ra
        << endl;
}

int sc_main(int, char**)
{
  const int N = 2000;
  int Seed = rng.rand();
  cout<<"\nverifying sc_bv<"<<N<<">" << endl;
  try{
    for(int i=0; i<1000; i++)
      compare<sc_bv<N> >(N);
  }
  catch(...)
  {
    cout<<"Test failed due to exception in sc_bv. Seed = "<<Seed<< endl;
    throw;
  }
  try{
    cout<<"\nverifying sc_lv<"<<N<<">" << endl;
    for(int i=0; i<1000; i++)
      compare<sc_lv<N> >(N);
  }
  catch(...)
  {
    cout<<"Test failed due to exception in sc_lv. Seed = "<<Seed<< endl;
    throw;
  }

  return 0;
}
