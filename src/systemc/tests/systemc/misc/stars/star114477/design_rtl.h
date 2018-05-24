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

  design_rtl.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc.h>

SC_MODULE(fun)    {
public:
  sc_in<sc_uint<1> > clk;
  sc_in<sc_uint<32> > count;
  sc_out<sc_uint<9> > out_a0;
  sc_out<sc_uint<9> > out_a1;
  SC_CTOR(fun)    {
    SC_METHOD(p1);
    sensitive << count << out_a1;
  };
private:
    
    template<int w> sc_int<w> int_conv_from_sc_uint_to_sc_int ( sc_uint<w> 
        a) {
      sc_int<w> a_temp;
      for (int i=0;i<w; i++)
        a_temp[i] = a[i];
      return a_temp;
    }
    
    template<int w> sc_bigint<w> int_conv_from_sc_biguint_to_sc_bigint ( 
        sc_biguint<w> a) {
      sc_bigint<w> a_temp;
      for (int i=0;i<w; i++)
        a_temp[i] = a[i];
      return a_temp;
    }
    
    template<int w> sc_int<w> int_conv_from_sc_uint_to_sc_int ( bool a) {
      sc_int<w> a_temp;
      a_temp[0] = a;
      return a_temp;
    }
    
    
    sc_uint<32> C18_B, C18_REMAINDER;
    bool N73, N74, N75, N76, N77, N78, N79, N80, N81, N82, N83, N84, N85, 
        N86, N87, N88, N89, N90, N91, N92, N93, N94, N95, N96, N97, N98, 
        N99, N100, N101, N102, N103, N104, N105, N106, N107, N108, N109, 
        N110, N111, N112, N113, N114, N115, N116, N117, N118, N119, N120, 
        N121, N122, N123, N124, N125, N126, N127, N128, N129, N130, N131, 
        N132, N133, N134;
    
    void p1( ) {
      out_a1[0].write( 0 );
      out_a1[1].write( 0 );
      out_a1[3].write( 1 );
      out_a1[5].write( 1 );
      out_a1[7].write( 0 );
      out_a1[8].write( 0 );
      out_a0[2].write( (0 ^ 1) );
      out_a0[5].write( (0 ^ 1) );
      C18_B[31] =  0 ;
      C18_B[30] =    0 ;
      C18_B[29] =    0 ;
      C18_B[28] =    0 ;
      C18_B[27] =    0 ;
      C18_B[26] =    0 ;
      C18_B[25] =    0 ;
      C18_B[24] =    0 ;
      C18_B[23] =    0 ;
      C18_B[22] =    0 ;
      C18_B[21] =    0 ;
      C18_B[20] =    0 ;
      C18_B[19] =    0 ;
      C18_B[18] =    0 ;
      C18_B[17] =    0 ;
      C18_B[16] =    0 ;
      C18_B[15] =    0 ;
      C18_B[14] =    0 ;
      C18_B[13] =    0 ;
      C18_B[12] =    0 ;
      C18_B[11] =    0 ;
      C18_B[10] =    0 ;
      C18_B[9] =    0 ;
      C18_B[8] =    0 ;
      C18_B[7] =    0 ;
      C18_B[6] =    0 ;
      C18_B[5] =    0 ;
      C18_B[4] =    0 ;
      C18_B[3] =    0 ;
      C18_B[2] =    0 ;
      C18_B[1] =    1 ;
      C18_B[0] =    1 ;
      
      // REM_UNS_OP(A,B,REMAINDER)
      REM_UNS_OP( count.read(), C18_B, C18_REMAINDER );
    }
    out_a0[7].write( (0 ^ 0) );
    out_a0[8].write( (0 ^ 0) );
    N73 =  !(C18_REMAINDER[31]);
    N74 =  !(C18_REMAINDER[30]);
    N76 =  !(C18_REMAINDER[29]);
    N78 =  !(C18_REMAINDER[28]);
    N80 =  !(C18_REMAINDER[27]);
    N82 =  !(C18_REMAINDER[26]);
    N84 =  !(C18_REMAINDER[25]);
    N86 =  !(C18_REMAINDER[24]);
    N88 =  !(C18_REMAINDER[23]);
    N90 =  !(C18_REMAINDER[22]);
    N92 =  !(C18_REMAINDER[21]);
    N94 =  !(C18_REMAINDER[20]);
    N96 =  !(C18_REMAINDER[19]);
    N98 =  !(C18_REMAINDER[18]);
    N100 =  !(C18_REMAINDER[17]);
    N102 =  !(C18_REMAINDER[16]);
    N104 =  !(C18_REMAINDER[15]);
    N106 =  !(C18_REMAINDER[14]);
    N108 =  !(C18_REMAINDER[13]);
    N110 =  !(C18_REMAINDER[12]);
    N112 =  !(C18_REMAINDER[11]);
    N114 =  !(C18_REMAINDER[10]);
    N116 =  !(C18_REMAINDER[9]);
    N118 =  !(C18_REMAINDER[8]);
    N120 =  !(C18_REMAINDER[7]);
    N122 =  !(C18_REMAINDER[6]);
    N124 =  !(C18_REMAINDER[5]);
    N126 =  !(C18_REMAINDER[4]);
    N128 =  !(C18_REMAINDER[3]);
    N130 =  !(C18_REMAINDER[2]);
    N132 =  !(C18_REMAINDER[1]);
    N134 =  !(C18_REMAINDER[0]);
    N75 =  (N73 && N74);
    N77 =  (N75 && N76);
    N79 =  (N77 && N78);
    N81 =  (N79 && N80);
    N83 =  (N81 && N82);
    N85 =  (N83 && N84);
    N87 =  (N85 && N86);
    N89 =  (N87 && N88);
    N91 =  (N89 && N90);
    N93 =  (N91 && N92);
    N95 =  (N93 && N94);
    N97 =  (N95 && N96);
    N99 =  (N97 && N98);
    N101 =  (N99 && N100);
    N103 =  (N101 && N102);
    N105 =  (N103 && N104);
    N107 =  (N105 && N106);
    N109 =  (N107 && N108);
    N111 =  (N109 && N110);
    N113 =  (N111 && N112);
    N115 =  (N113 && N114);
    N117 =  (N115 && N116);
    N119 =  (N117 && N118);
    N121 =  (N119 && N120);
    N123 =  (N121 && N122);
    N125 =  (N123 && N124);
    N127 =  (N125 && N126);
    N129 =  (N127 && N128);
    N131 =  (N129 && N130);
    N133 =  (N131 && N132);
    out_a1[6].write( (N133 && N134) );
    out_a1[4].write( out_a1.read()[6] );
    out_a0[3].write( (0 ^ out_a1.read()[6]) );
    out_a0[4].write( (0 ^ out_a1.read()[6]) );
    out_a0[6].write( (0 ^ out_a1.read()[6]) );
    out_a1[2].write( !(out_a1.read()[6]) );
    out_a0[0].write( (out_a1.read()[2] ^ 0) );
    out_a0[1].write( (out_a1.read()[6] ^ out_a1.read()[2]) );
  
  }

};
