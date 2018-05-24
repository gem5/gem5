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

#include <systemc.h>


SC_MODULE(xyz) {
  
    sc_in<sc_int<8> > A;
    sc_out<sc_lv<8> > X;

    void convert(const sc_lv<8>& I, sc_int<8>& O) {
	O=I;
    }

    void convert2(const sc_int<8>& I, sc_lv<8>& O) {
	O=I;
    }

    void entry() {
	sc_int<8> tmp;
	sc_lv<8> tmp2;

	convert(sc_lv<8>(A.read()),tmp);
	convert2(tmp,tmp2);
	X.write(tmp2);
    }

    SC_CTOR(xyz) {
	SC_METHOD(entry);
	sensitive << A;
    }
};


#define NS * 1e-9

int sc_main(int ac, char *av[])
{
  //Signals
  sc_signal<sc_int<8> > A;
  sc_signal<sc_lv<8> > X;

  xyz UUT("UUT");
  UUT.A(A);
  UUT.X(X);

  sc_start(0, SC_NS);

  A.write(1);
  sc_start( 10, SC_NS );
  cout << X.read() << endl;

  A.write(2);
  sc_start( 10, SC_NS );
  cout << X.read() << endl;

  A.write(3);
  sc_start( 10, SC_NS );
  cout << X.read() << endl;

  return 0;
}
