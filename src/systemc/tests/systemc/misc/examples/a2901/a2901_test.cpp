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

  a2901_test.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "a2901_test.h"

void
a2901_test::entry()
{
      switch (vec_cnt++) {
      case 0:
	I.    write(0x7);
	D.    write(  0);
	C0.   write(  0);
	OEbar.write(  0);
	break;
      case 1:
	I.    write(0x46);
	D.    write(   0);
	C0.   write(   0);
	OEbar.write(   0);
	break;
      case 2:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000 (%0d)\n", (int)Y.read());
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1 (%0d)\n", (int)F30.read());
	
	I.write(0x7);
	D.write(0x0);
	C0.write(0x0);
	OEbar.write(0x0);
	
	break;
      case 3:
	I.write(0x46);
	D.write(0x0);
	C0.write(0x1);		
	OEbar.write(0x0);
	break;
      case 4:
	if (Y.read() != 0x1) fprintf(stderr,"Y != 4'b0001\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I.write(0x7);
	D.write(0xf);					// N("4'b1111")
	C0.write(0x0);
	OEbar.write(0x0);

	break;
      case 5:
	I.write(0x46);
	D.write(0x0);
	C0.write(0x0);
	OEbar.write(0x0);

	break;
      case 6:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I.write(0x7);
	D.write(0xf);
	C0.write(0x0);
	OEbar.write(0x0);

	break;
      case 7:
	I.write(0x46);
	D.write(0x0);
	C0.write(0x1);
	OEbar.write(0x0);
	break;
      case 8:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");
	
	I.write(0x7);
	D.write(0xf);
	C0.write(0x0);
	OEbar.write(0x0);
	break;
      case 9:
	I.write(0x46);
	D.write(0xf);
	C0.write(0x0);
	OEbar.write(0x0);
	break;
      case 10:
	if (Y.read() != 0xe) fprintf(stderr,"Y != 4'b1110\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x0) fprintf(stderr,"Gbar != 1'b0\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I.write(0x7);
	D.write(0xf);
	C0.write(0x0);
	OEbar.write(0x0);
	break;
      case 11:
	I.write(0x46);
	D.write(0xf);
	C0.write(0x1);
	OEbar.write(0x0);
	break;
      case 12:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x0) fprintf(stderr,"Gbar != 1'b0\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 13:
	I .write( 0x46);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 14:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 15:
	I .write( 0x46);
	D .write( 0xf);
	C0 .write( 0x1);
	OEbar .write( 0x0);
	break;
      case 16:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");
	
	I .write( 0x7);
	D .write( 0x1);				       
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 17:
	I .write( 0x6);	
	D .write( 0x1);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 18:
	if (Y.read() != 0x2) fprintf(stderr,"Y != 4'b0010\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0x2);					// N("4'b0010")
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 19:
	I .write( 0x46);
	D .write( 0x2);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 20:
	if (Y.read() != 0x4) fprintf(stderr,"Y != 4'b0100\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0x4);					
	C0 .write( 0x0);
	OEbar .write( 0x0);
	
	break;
      case 21:
	I .write( 0x46);
	D .write( 0x4);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 22:
	if (Y.read() != 0x8) fprintf(stderr,"Y != 4'b1000\n");
	if (C4.read()!= 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x1) fprintf(stderr,"OVR != 1'b1\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0x8);					// N("4'b1000")
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 23:
	I .write( 0x46);
	D .write( 0x8);
	C0 .write( 0x0);
	OEbar .write( 0x0);

	break;
      case 24:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x0) fprintf(stderr,"Gbar != 1'b0\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x1) fprintf(stderr,"OVR != 1'b1\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");

	// ******** SUBTRACTION S - R ********
	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 25:
	I .write( 0x4e);					// N("9'b001001110")
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	
	break;
      case 26:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");

	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 27:
	I .write( 0x4e);
	D .write( 0x0);
	C0 .write( 0x1);
	OEbar .write( 0x0);
	break;
      case 28:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");

	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 29:
	I .write( 0x4e);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);	
	break;
      case 30:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");
	
	I .write( 0x7);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 31:
	I .write( 0x4e);
	D .write( 0xf);
	C0 .write( 0x1);
	OEbar .write( 0x0);

	break;
      case 32:
	if (Y.read() != 0x1) fprintf(stderr,"Y != 4'b0001\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x1) fprintf(stderr,"Pbar != 1'b1\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	
	break;
      case 33:
	I .write( 0x4e);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	
	break;
      case 34:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x0) fprintf(stderr,"C4 != 1'b0\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);	
	break;
      case 35:
	I .write( 0x4e);
	D .write( 0xf);
	C0 .write( 0x1);
	OEbar .write( 0x0);
	
	break;
      case 36:
	if (Y.read() != 0x0) fprintf(stderr,"Y != 4'b0000\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x1) fprintf(stderr,"Gbar != 1'b1\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x0) fprintf(stderr,"F3 != 1'b0\n");
	if (F30.read() != 0x1) fprintf(stderr,"F30 != 1'b1\n");
	
	I .write( 0x7);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);

	break;
      case 37:
	I .write( 0x4e);
	D .write( 0x0);
	C0 .write( 0x0);
	OEbar .write( 0x0);

	break;
      case 38:
	if (Y.read() != 0xe) fprintf(stderr,"Y != 4'b1110\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x0) fprintf(stderr,"Gbar != 1'b0\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");
	
	I .write( 0x7);
	D .write( 0xf);
	C0 .write( 0x0);
	OEbar .write( 0x0);
	break;
      case 39:
	I .write( 0x4e);
	D .write( 0x0);
	C0 .write( 0x1);
	OEbar .write( 0x0);
	break;
      case 40:
	if (Y.read() != 0xf) fprintf(stderr,"Y != 4'b1111\n");
	if (C4.read() != 0x1) fprintf(stderr,"C4 != 1'b1\n");
	if (Gbar.read() != 0x0) fprintf(stderr,"Gbar != 1'b0\n");
	if (Pbar.read() != 0x0) fprintf(stderr,"Pbar != 1'b0\n");
	if (OVR.read() != 0x0) fprintf(stderr,"OVR != 1'b0\n");
	if (F3.read() != 0x1) fprintf(stderr,"F3 != 1'b1\n");
	if (F30.read() != 0x0) fprintf(stderr,"F30 != 1'b0\n");

	I .write( 0x7);
	D .write( 0x0); // 0x1;
	C0 .write( 0x0);
	OEbar .write( 0x0);
	
	if (++loop < 100000)
	  vec_cnt = 0;
	else {
          printf ("loops = %d\n", loop);
	  sc_stop();
        }
	break;
      }
}
