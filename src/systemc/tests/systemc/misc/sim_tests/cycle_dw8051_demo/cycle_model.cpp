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

  cycle_model.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

//***************************************************************************
// FILE: cycle_model.cc
// 
// AUTHOR: Luc Semeria    September, 21, 1998
//
// ABSTRACT: cycle-accurate model based on the dw8051 architecture
//             
//
// MODIFICATION HISTORY:
//         Luc Semeria: 21/9/98 created
//
//***************************************************************************
// 
// DESCRIPTION
// 
// During initialization, the model parses the Intel hex file and put the 
// program into memory.
// Then the cycle-accurate model does the following operations:
//
//   main loop:
//     fetch instruction
//     decode instruction
//     execute instruction                   /read instr mem
//              |\- fetch operand 1 (and 2) <             /mem bus access
//              |                            \fetch data <
//              |                                         \read data mem
//              |\- execute operation
//              |                     /mem bus access
//              |\- write back data  <
//              |                     \write data mem
//              |
//               \- compute next address
//                                             
//    
// The external instruction and data memories are part of the model
// so these memory accesses are just read and write in internal memory
// The simulation is then speeded up because no bus transactions occurs.
// The model doesn't switch from one process to another.
//
// To communicate with peripheral on the memory bus, the bus can be 
// used and the model automatically switches to a real cycle-accurate mode 
// for a given number of cycle. This is implemented within the function:
//   request_address(int addr);
// 
// This cycle-accurate model implements only parts of the dw8051. The 
// limitations are the following:
//       - some instructions are not supported (cf decode function)
//       - SFR, timers, io_interface and interrupts are not supported
//
//***************************************************************************

#include "cycle_model.h"
#include <string.h>

/* useful macros for sc_aproc */
#define AT_POSEDGE(CLK) wait(); while(!clk.posedge()) wait();
#define AT_NEGEDGE(CLK) wait(); while(!clk.negedge()) wait();

bool ALL_CYCLES;         /* flag to execute all cycles */


//-------------------------------------------------------------------------
// void cycle_model::parse_hex(char *name) 
//
// parse Intel HEX file
// more information on hex format on-line at:
//         http://www.8052.com/tutintel.htm
//
//------------------------------------------------------------------------
void cycle_model::parse_hex(char *name) {
  char line_buffer[MEM_SIZE];
  FILE *hex_file;

  // open file
  hex_file = fopen(name,"r");
  if(hex_file==NULL) {
    fprintf(stderr,"Error in opening file %s\n",name);
    exit(-1);
  }

  // read new line at each loop ------------------------------------------
  while(fgets(line_buffer,MEM_SIZE,hex_file)!=NULL) {  
#ifdef DEBUG
    printf("Read new line -> %s",line_buffer);
#endif

    // parse line --------------------------------------------------------
    
    // parse ':' (line[0])
    if(line_buffer[0]!=':') {
      continue;
    }
        
    
    // parse length (line[1..2])
    int length;
    // char length_string[2];
    char length_string[3];
    if(strncpy(length_string,&(line_buffer[1]),2)==NULL) {
      fprintf(stderr,"Error in parsing length\n");
      exit(-1);
    }
    length_string[2] = 0;
    length = (int)strtol(length_string, (char **)NULL, 16);
#ifdef DEBUG
    printf("length=%x\n",length);
#endif

    // parse address (line[3..6])
    int address;
    // char address_string[4];
    char address_string[5];
    if(strncpy(address_string,&(line_buffer[3]),4)==NULL) {
      fprintf(stderr,"Error in parsing address\n");
      exit(-1);
    }
    address_string[4] = 0;
    address = (int)strtol(address_string, (char **)NULL, 16);
#ifdef DEBUG
    printf("address=%x\n",address);
#endif


    // parse Record Type (line[7..8])
    int record_type;
    // char record_string[2];
    char record_string[3];
    if(strncpy(record_string,&(line_buffer[7]),2)==NULL) {
      fprintf(stderr,"Error in parsing record type\n");
      exit(-1);
    }
    record_string[2] = 0;
    record_type = (int)strtol(record_string, (char **)NULL, 16);
#ifdef DEBUG
    printf("record_type=%x\n",record_type);
#endif
    if(record_type==01) {
      // end of file
      // return;
#ifdef DEBUG
      printf("end of file => return\n");
#endif
      fclose(hex_file);
      return;
    }

    // parse data bytes
    char instr_string[3];
    for(int i=0;i<length;i++) {
      if(strncpy(instr_string,&(line_buffer[2*i+9]),2)==NULL) {
	fprintf(stderr,"Error in parsing data byte %d\n",i);
	exit(-1);
      }

    instr_string[2] = 0;
    int temp = (int)strtol(instr_string, (char **)NULL, 16);
    instr_mem[address++] = temp;
#ifdef DEBUG
    printf("data byte = %x\n",temp);
#endif
    }
    
    // skip the checksum bits

    // verify end of line
    if(line_buffer[2*length+9+2]!='\n') {
      fprintf(stderr,"Error in parsing hex file: end of line expected\n");
      exit(-1);
    }
    
  }
  
  fprintf(stderr,"Error in parsing hex file: end of file record type expected\n");
  exit(-1);

}


//---------------------------------------------------------------------
//void cycle_model::decode(int opcode, instr* i)
//
// take an opcode as an input and output the instruction with the
// proper operand types.
//
//---------------------------------------------------------------------
void cycle_model::decode(int opcode, instr* i) {

  // default 
  i->type = i_nop;
  i->n_src = 0;
  i->src1.type = o_null;
  i->src1.val = -1;
  i->src2.type = o_null;
  i->src2.val = -1;
  i->dst.type = o_null;
  i->dst.val = -1;
  
  switch(opcode) { 
    // arithmetic operations -----------------------------------------
  case 0x28:
  case 0x29:
  case 0x2A:
  case 0x2B:
  case 0x2C:
  case 0x2d:
  case 0x2e:
  case 0x2f: {
    // add register to A
    i->type = i_add;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x25: {
    // add direct byte to A
    i->type = i_add;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x26:
  case 0x27: {
    // add data memory to A
    i->type = i_add;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x24: {
    // add immediate to A
    i->type = i_add;
    i->n_src = 2;
    i->src1.type = o_cst;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x98:
  case 0x99:
  case 0x9A:
  case 0x9B:
  case 0x9C:
  case 0x9d:
  case 0x9e:
  case 0x9f: {
    // sub register to A
    i->type = i_sub;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x95: {
    // sub direct byte to A
    i->type = i_sub;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x96:
  case 0x97: {
    // sub data memory to A
    i->type = i_sub;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x94: {
    // sub immediate to A
    i->type = i_sub;
    i->n_src = 2;
    i->src1.type = o_cst;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x04: {
    // increment A
    i->type = i_inc;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x08:
  case 0x09:
  case 0x0A:
  case 0x0B:
  case 0x0C:
  case 0x0d:
  case 0x0e:
  case 0x0f: {
    // increment register
    i->type = i_inc;
    i->n_src = 1;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->dst.type = o_reg;
    i->dst.val = opcode&0x07;
    i->cycle = 1;
    break;
  }
  case 0x05: {
    // increment direct byte
    i->type = i_inc;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x06:
  case 0x07: {
    // increment  data memory
    i->type = i_inc;
    i->n_src = 1;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->dst.type = o_ind;
    i->dst.val = opcode&1;
    i->cycle = 1;
    break;
  }
  case 0x14: {
    // decrement A
    i->type = i_dec;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x18:
  case 0x19:
  case 0x1A:
  case 0x1B:
  case 0x1C:
  case 0x1d:
  case 0x1e:
  case 0x1f: {
    // decrement register
    i->type = i_dec;
    i->n_src = 1;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->dst.type = o_reg;
    i->dst.val = opcode&0x07;
    i->cycle = 1;
    break;
  }
  case 0x15: {
    // decrement direct byte
    i->type = i_dec;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x16:
  case 0x17: {
    // increment  data memory
    i->type = i_dec;
    i->n_src = 1;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->dst.type = o_ind;
    i->dst.val = opcode&1;
    i->cycle = 1;
    break;
  }
  // logic operation --------------------------------------------------
  case 0x58:
  case 0x59:
  case 0x5A:
  case 0x5B:
  case 0x5C:
  case 0x5d:
  case 0x5e:
  case 0x5f: {
    // and register to A
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x55: {
    // and direct byte to A
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x56:
  case 0x57: {
    // and data memory to A
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x54: {
    // and immediate to A
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_cst;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x52: {
    // and A to direct byte
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x53: {
    // and immdiate to direct byte
    i->type = i_and;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_cst;
    i->dst.type = o_dir;
    i->cycle = 3;
    break;
  }
  case 0x48:
  case 0x49:
  case 0x4A:
  case 0x4B:
  case 0x4C:
  case 0x4d:
  case 0x4e:
  case 0x4f: {
    // or register to A
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x45: {
    // or direct byte to A
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x46:
  case 0x47: {
    // or data memory to A
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x44: {
    // or immediate to A
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_cst;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x42: {
    // or A to direct byte
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x43: {
    // or immediate to direct byte
    i->type = i_or;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_cst;
    i->dst.type = o_dir;
    i->cycle = 3;
    break;
  }
  case 0x68:
  case 0x69:
  case 0x6A:
  case 0x6B:
  case 0x6C:
  case 0x6d:
  case 0x6e:
  case 0x6f: {
    // xor register to A
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x65: {
    // xor direct byte to A
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x66:
  case 0x67: {
    // xor data memory to A
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x64: {
    // xor immediate to A
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_cst;
    i->src2.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0x62: {
    // and A to direct byte
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_acc;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x63: {
    // xor immdiate to direct byte
    i->type = i_xor;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_cst;
    i->dst.type = o_dir;
    i->cycle = 3;
    break;
  }
  case 0xf4: {
    // complement A
    i->type = i_cpl;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x23: {
    // rotate A left
    i->type = i_rl;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x03: {
    // rotate A right
    i->type = i_rr;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  // data transfer -----------------------------------------------
  case 0xe8:
  case 0xe9:
  case 0xeA:
  case 0xeB:
  case 0xeC:
  case 0xed:
  case 0xee:
  case 0xef: {
    // move register to A
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0xe5: {
    // move direct bit to A
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0xe6:
  case 0xe7: {
    // move data memory to A
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_ind;
    i->src1.val = opcode&1;
    i->dst.type = o_acc;
    i->cycle = 1;
    break;
  }
  case 0x74: {
    // move immediate to A
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_cst;
    i->dst.type = o_acc;
    i->cycle = 2;
    break;
  }
  case 0xf8:
  case 0xf9:
  case 0xfA:
  case 0xfB:
  case 0xfC:
  case 0xfd:
  case 0xfe:
  case 0xff: {
    // move A to register
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_reg;
    i->dst.val = opcode&0x07;
    i->cycle = 1;
    break;
  }
  case 0xa8:
  case 0xa9:
  case 0xaA:
  case 0xaB:
  case 0xaC:
  case 0xad:
  case 0xae:
  case 0xaf: {
    // move direct to register
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_reg;
    i->dst.val = opcode&0x07;
    i->cycle = 2;
    break;
  }
  case 0x78:
  case 0x79:
  case 0x7A:
  case 0x7B:
  case 0x7C:
  case 0x7d:
  case 0x7e:
  case 0x7f: {
    // move immediate to register
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_cst;
    i->dst.type = o_reg;
    i->dst.val = opcode&0x07;
    i->cycle = 2;
    break;
  }
  case 0xf5: {
    // move A to direct byte
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_dir;
    i->cycle = 2;
  }
  case 0x88:
  case 0x89:
  case 0x8A:
  case 0x8B:
  case 0x8C:
  case 0x8d:
  case 0x8e:
  case 0x8f: {
    // move register to direct byte
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_reg;
    i->src1.val = opcode&0x07;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x85: {
    // move direct byte to direct byte
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_dir;
    i->cycle = 3;
    break;
  }
  case 0x86:
  case 0x87: {
    // move data memory to direct byte
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_ind;
    i->src1.val = opcode&0x01;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
  }
  case 0x75: {
    // move immediate to direct byte
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_cst;
    i->dst.type = o_dir;
    i->cycle = 2;
    break;
    }
  case 0xf6:
  case 0xf7: {
    // move A to data memory
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_ind;
    i->dst.val = opcode&1;
    i->cycle = 1;
    break;
  }
  case 0xa6:
  case 0xa7: {
    // move direct byte to data memory
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_dir;
    i->dst.type = o_ind;
    i->dst.val = opcode&1;
    i->cycle = 2;
    break;
  }
  case 0x76:
  case 0x77: {
    // move immediate to data memory
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_cst;
    i->dst.type = o_ind;
    i->dst.val = opcode&1;
    i->cycle = 2;
    break;
  }
  case 0xe2:
  case 0xe3: {
    // move external data to A
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_ext;
    i->src1.val = opcode&1;
    i->dst.type = o_acc;
    i->cycle = 2+stretch_cycles;
    break;
  }
  case 0xf2:
  case 0xf3: {
    // move A to external data
    i->type = i_mov;
    i->n_src = 1;
    i->src1.type = o_acc;
    i->dst.type = o_ext;
    i->dst.val = opcode&1;
    i->cycle = 2+stretch_cycles;
    break;
  }
  // branching ----------------------------------------------------
  case 0x11:
  case 0x31:
  case 0x51:
  case 0x71:
  case 0x91:
  case 0xb1:
  case 0xd1:
  case 0xf1: {
    // absolute call to subroutine
    i->type = i_call;
    i->n_src = 1;
    i->src1.type = o_add;
    i->src1.val = (opcode>>5)&7;
    i->cycle = 3;
    break;
  }
  case 0x12: {
    // Long call to subroutine
    i->type = i_call;
    i->n_src = 1;
    i->src1.type = o_ladd;
    i->cycle = 4;
    break;
  }
  case 0x22: {
    // return from subroutine
    i->type = i_ret;
    i->cycle = 4;
    break;
  }
  case 0x01:
  case 0x21:
  case 0x41:
  case 0x61:
  case 0x81:
  case 0xa1:
  case 0xc1:
  case 0xe1: {
    // absolute jump unconditional
    i->type = i_jmp;
    i->n_src = 1;
    i->src1.type = o_add;
    i->src1.val = (opcode>>5)&7;
    i->cycle = 3;
    break;
  }
  case 0x02: {
    // Long jump unconditional
    i->type = i_jmp;
    i->n_src = 1;
    i->src1.type = o_ladd;
    i->cycle = 4;
    break;
  }
  case 0x60: {
    // jump on accumulator = 0
    i->type = i_jz;
    i->n_src = 1;
    i->src1.type = o_rel;
    i->cycle = 3;
    break;
  }
  case 0x70: {
    // jump on accumulator != 0
    i->type = i_jnz;
    i->n_src = 1;
    i->src1.type = o_rel;
    i->cycle = 3;
    break;
  }
  case 0xb5: {
    // compare A,direct JNE
    i->type = i_cjne;
    i->n_src = 2;
    i->src1.type = o_acc;
    i->src2.type = o_dir;
    i->dst.type = o_rel;
    i->cycle = 4;
    break;
  }
  case 0xb4: {
    // compare A,immeditate JNE
    i->type = i_cjne;
    i->n_src = 2;
    i->src1.type = o_acc;
    i->src2.type = o_cst;
    i->dst.type = o_rel;
    i->cycle = 4;
    break;
  }
  case 0xB8:
  case 0xB9:
  case 0xBa:
  case 0xBb:
  case 0xBc:
  case 0xBd:
  case 0xBe:
  case 0xBf: {
    // compare reg,immeditate JNE
    i->type = i_cjne;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode & 0x7;
    i->src2.type = o_cst;
    i->dst.type = o_rel;
    i->cycle = 4;
    break;
  }
  case 0xb6:
  case 0xb7: {
    // compare memory byte,immeditate JNE
    i->type = i_cjne;
    i->n_src = 2;
    i->src1.type = o_ind;
    i->src1.val = opcode & 0x1;
    i->src2.type = o_cst;
    i->dst.type = o_rel;
    i->cycle = 4;
    break;
  }
  case 0xd8:
  case 0xd9:
  case 0xda:
  case 0xdb:
  case 0xdc:
  case 0xdd:
  case 0xde:
  case 0xdf: {
    // decrement reg, JNZ relative
    i->type = i_djnz;
    i->n_src = 2;
    i->src1.type = o_reg;
    i->src1.val = opcode & 0x7;
    i->src2.type = o_rel;
    i->cycle = 3;
    break;
  }
  case 0xd5: {
    // decrement direct byte, JNZ relative
    i->type = i_djnz;
    i->n_src = 2;
    i->src1.type = o_dir;
    i->src2.type = o_rel;
    i->cycle = 4;
    break;
  }
  // NOP --------------------------------------------------------------
  case 0x00: {
    break;
  }
  default: {
    
    break;
    fprintf(stderr,"opcode 0x%x not supported\n",opcode);
    break;
  }
  }

#ifdef DEBUG
  printf("decode instr type:%d, src1: %d, src2: %d, dest %d, nb_cycles: %d\n",i->type, i->src1.type, i->src2.type, i->dst.type, i->cycle);
#endif
}


//--------------------------------------------------------------------
// bool request_address(int ad); 
//      
//    return 0 if the memory adress is external (i.e. external peripheral)   
//    update cycles2execute so that the simulation runs for a given
//    number of clock cycles.
//
//--------------------------------------------------------------------
bool cycle_model::request_address(int ad) {
  // add peripheral driver here
  //
  // if(ad==<ADDRESS OF THE PERIPH>) {
  //    if(cycles2execute<=<NB_CYCLES>)
  //        cycles2execute = <NB_CYCLES>;
  //    return 0;
  // }
    
  if(ad==0x10) {
    if(cycles2execute<=30)
      cycles2execute = 30;
    return 0;
  }
  
  if(ad==0x11) {
    return 0;
  }
  
  return 1;
}


//--------------------------------------------------------------------
// exec_bus_cycle(bus_cycle_type op, int addr, int data, int* result)
//
//    executes a bus cycle (IDLE, MEM_READ, MEM_WRITE).
//       - IDLE: executes an idle cycle (4 clocks)
//       - MEM_READ: reads from the memory bus (stretch+1 clocks)
//       - MEM_WRITE: writes on the memory bus (stretch+1 clocks)
//
//--------------------------------------------------------------------
void cycle_model::exec_bus_cycle(bus_cycle_type op, int addr, int data, int* result) {
  int cycles = 0;
  int mem_idle =0;

  
  if(op==OP_IDLE) {
    // OP_IDLE
    if((cycles2execute>0)||ALL_CYCLES) {
      // wait 4 cycles
      mem_ale.write(0);
      mem_wr_n.write(1);
      mem_pswr_n.write(1);
      mem_rd_n.write(1);
      mem_psrd_n.write(1);
      p0_mem_reg_n.write(0);
      p0_addr_data_n.write(0);
      AT_POSEDGE(clk);  
      AT_POSEDGE(clk); 
      AT_POSEDGE(clk); 
      AT_POSEDGE(clk);
      cycles2execute -= 1;
    }
    
    cycle_count += 1;
    return;
  }

  
  // OP_MEM_READ or OP_MEM_WRITE
  do {
    cycles++;
    
    // Cycle 1 *********************************************************
    if(mem_idle==0) {
      mem_ale.write(1);
      mem_wr_n.write(1);
      mem_pswr_n.write(1);
      mem_rd_n.write(1);
      mem_psrd_n.write(1);
      p0_mem_reg_n.write(0);
      p0_addr_data_n.write(0);
      
      if(op==OP_MEM_WRITE) {
	mem_data_out.write( sc_bv<8>( data ) );
	p0_mem_reg_n.write(1);
	p0_addr_data_n.write(1);
      }
    }
    
    AT_POSEDGE(clk);
    
    
    // Cycle 2 *********************************************************
    if(mem_idle==0) {
      switch (op) {
      case OP_MEM_READ: {
	mem_addr.write( sc_bv<16>( addr & 0x0000ffff ) );
	p0_mem_reg_n.write(1);
	p0_addr_data_n.write(1);
	p2_mem_reg_n.write(1);
	break;
      }
      case OP_MEM_WRITE: {
	mem_addr.write( sc_bv<16>( addr & 0x0000ffff ) );
	p0_addr_data_n.write(0);
	p2_mem_reg_n.write(1);
	break;
      }	
      default: {
	// do nothing
	break;
      }
      }
    }
    if(mem_idle==0) { 
      AT_NEGEDGE(clk);
      mem_ale.write(0);
    }
    
    AT_POSEDGE(clk);
    
    
    // Cycle 3 *********************************************************
    if(mem_idle==0) {
      switch (op) {
      case OP_MEM_READ: {
	p0_mem_reg_n.write(0);
	p0_addr_data_n.write(0);
	
	if(stretch_cycles==0)
	  mem_rd_n.write(0); // read RAM
	break;
      }
      case OP_MEM_WRITE: {
	if(stretch_cycles==0)
	  mem_wr_n.write(0); // write RAM
	break;
      }	
      default: {
	// do nothing
	break;
      }
      }
    }
    AT_POSEDGE(clk);
    
    
    // Cycle 4 *********************************************************
    if (mem_idle==0) {
      switch (op) {
      case OP_MEM_READ: {
	if(stretch_cycles>0) {
	  mem_idle=stretch_cycles+1;
	  mem_rd_n.write(0); // read RAM
	}
	break;
      }
      case OP_MEM_WRITE: {
	if(stretch_cycles>0) {
	  mem_idle=stretch_cycles+1;
	  mem_wr_n.write(0); // write RAM
	}
	break;
      }
      default: {
	// do nothing 
	break;
      }
      }
    }
    else if(mem_idle==1) {
      // read/write enable <- 1 when stretch>0
      switch (op) {
      case OP_MEM_READ: {
	if(stretch_cycles>0) {
	  // read value
	  *result = mem_data_in.read().to_uint();
	  // reset read enable
	  mem_rd_n.write(1); // read RAM
	}	
	break;
      }
      case OP_MEM_WRITE: {
	if(stretch_cycles>0) {
	  // reset write enable
	  mem_wr_n.write(1); // write RAM
	}
	break;
      }
      default: {
	break;
      }
      }
    }
    AT_POSEDGE(clk);
    
    
    // Cycle 1 (1st part) **********************************************
    if(mem_idle>0)
      mem_idle--;
    
    if(mem_idle==0){ 
      switch(op) {
      case OP_MEM_READ:
	if(stretch_cycles==0) {
	  mem_rd_n.write(1);
	  *result = mem_data_in.read().to_uint();
	}
	break;
      case OP_MEM_WRITE:
	if(stretch_cycles==0) 
	  mem_wr_n.write(1);
	break;
      default:
	break;
      }
    }
  } while(mem_idle>0);
  
  sc_assert(cycles==(stretch_cycles+1));
  cycle_count += cycles;
  cycles2execute-=cycles;
  return;
}




//------------------------------------------------------------------------
// int cycle_model::fetch_instr(int ad)
//
//    fetches data (1byte) from instruction memory
//
//------------------------------------------------------------------------
int cycle_model::fetch_instr(int ad) {

  sc_assert((ad<MEM_SIZE)&&(ad>=0));
  
  int temp;
  exec_bus_cycle(OP_IDLE, 0,0, &temp);

  int opcode = instr_mem[ad];
#ifdef DEBUG
  printf("Fetch instruction @0x%x (= 0x%x)\n",ad,opcode);
#endif
  return opcode;
}

//------------------------------------------------------------------------
// int cycle_model::fetch_data(int ad)
//
//   fetches data from memory which can be internal to the block or 
//   external (case of an hardware peripheral)
//
//------------------------------------------------------------------------
int cycle_model::fetch_data(int addr) {
  
  int data = 0, result;
  
  bool is_internal = request_address(addr);
  if(is_internal) {
    // is internal
    if((cycles2execute>0)||ALL_CYCLES) {
      // Wait 
      for(int i=0; i<stretch_cycles+1; i++) {
	exec_bus_cycle(OP_IDLE,addr,data,&result);
      }
    }
    result = ext_mem[addr];
  } else {
    // is external
    exec_bus_cycle(OP_MEM_READ,addr,data,&result);
  }
  
  return result;
}



//------------------------------------------------------------------------
// int cycle_model::write_data(int addr, int data)
//
//    writes data on data memory which can be internal to the block or
//    external (case of an hardware peripheral)
//
//------------------------------------------------------------------------
int cycle_model::write_data(int addr, int data) {
  
  int result = 0;
  
  bool is_internal = request_address(addr);
  
  if(is_internal) {
    // is internal
    if((cycles2execute>0)||ALL_CYCLES) {
      for(int i=0; i<stretch_cycles+1; i++) {
	exec_bus_cycle(OP_IDLE,addr,data,&result);
      }
    }
    ext_mem[addr]=data;
  } else {
    // is external
    exec_bus_cycle(OP_MEM_WRITE,addr,data,&result);
  }

  return result;
}


//--------------------------------------------------------------------
// int cycle_model::fetch_operand(operand* op)
//
//   returns the value of the operand
//
//--------------------------------------------------------------------
int cycle_model::fetch_operand(operand* op) {
  switch(op->type) {
  case o_acc: {
    return A;
    break;
  }
  case o_reg: {
    sc_assert((op->val<8)&&(op->val>=0));
#ifdef DEBUG
    printf("read R%d=%d\n",op->val,R[op->val]); 
#endif
    return R[op->val];
    break;
  }
  case o_dir: {
    // fetch address
    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);
    sc_assert((op->val<INT_SIZE)&&(op->val>=0));
    return int_mem[temp];
    break;
  }
  case o_ind: {
    sc_assert((op->val==0)||(op->val==1));
    sc_assert((R[op->val]<INT_SIZE)&&(R[op->val]>=0));
    return int_mem[R[op->val]];
    break;
  }
  case o_ext: {
    sc_assert((op->val==1)||(op->val==0));
    int addr = R[op->val];
    sc_assert((addr<MEM_SIZE)&&(addr>=0));
    
    int result = fetch_data(addr);

    return result;
    break;
  }
  case o_cst: {
    // fetch next byte
    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);
    return temp;
    break;
  }
  case o_lcst: {
    // fetch next 2 bytes

    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);

    my_stack->address += 1;
    sc_assert(my_stack->address<=MEM_SIZE);
    temp = (temp<<8) +  fetch_instr(my_stack->address);

    return temp;
    break;
  }
  case o_add: {
    // fetch next byte
    my_stack->address += 1;
    int temp = ((op->val)<<8) + fetch_instr(my_stack->address);
    return temp;
    break;
  }	
  case o_ladd: {
    // fetch next 2 bytes
    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);

    my_stack->address += 1;
    temp = (temp<<8) + fetch_instr(my_stack->address);
    return temp;
    break;
  }
  case o_rel: {
    // fetch next byte
    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);
    if(temp<0x80)
      return temp;
    else
      return -(0x100-temp);
    break;
  }
  default: {
    return -1;
    break;
  }
  }
  return -1;
}

//--------------------------------------------------------------------
// int write_back(operand *op, int value)
//
//    write the value into the operand
//
//--------------------------------------------------------------------
int cycle_model::write_back(operand* op, int v) {
  switch(op->type) {
  case o_acc: {
    A = v;
    return A;
    break;
  }
  case o_reg: {
    sc_assert((op->val<8)&&(op->val>=0));
    R[op->val] = v;
#ifdef DEBUG
    printf("write R%d <- %d\n",op->val,R[op->val]);
#endif
    return R[op->val];
    break;
  }
  case o_dir: {
    // write address
    my_stack->address += 1;
    int temp = fetch_instr(my_stack->address);
    sc_assert((temp<INT_SIZE)&&(temp>=0));
    int_mem[temp] = v;
    return int_mem[temp];
    break;
  }
  case o_ind: {
    sc_assert((op->val==0)||(op->val==1));
    sc_assert((R[op->val]<INT_SIZE)&&(R[op->val]>=0));
    int_mem[R[op->val]] = v;
    return int_mem[R[op->val]];
    break;
  }
  case o_ext: {
    sc_assert((op->val==1)||(op->val==0));
    int addr = R[op->val];
    sc_assert((addr<MEM_SIZE)&&(addr>=0));
    int data, result;
    data = v;
    result = write_data(addr,data);
    return result;
    break;
  }
  default: {
    return -1;
    break;
  }
  }
  return -1;
}



//--------------------------------------------------------------------
// void execute(instr *i)
//
//   execute consists of the following tasks:
//        - fetch the operands
//        - execute the operation in the intruction
//        - write the data back in the destination
//        - compute the next address for (jmp, call, return...)
//
//--------------------------------------------------------------------
void cycle_model::execute(instr *i) {
  int in1, in2, out = 0;

  // fetch operands ---------------------------------------------------
  if(i->n_src>=1)
    in1 = fetch_operand(&(i->src1));

  if(i->n_src>=2)
    in2 = fetch_operand(&(i->src2));
 
#ifdef DEBUG
  printf("execute %d, with in1=%d and in2=%d\n",i->type,in1, in2); 
#endif

  // execute ----------------------------------------------------------
  switch(i->type) {
  case i_add: {
    out = in1 + in2;
    break;
  }
  case i_sub: {
    out = in1 - in2;
    break;
  }	
  case i_inc: {
    out = in1+1;
    break;
  }
  case i_dec: {
    out = in1-1;
    break;
  }
  case i_mul: {
    out = in1 * in2;
    break;
  }
  case i_div: {
    out = in1/in2;
    break;
  }
  // logic operations
  case i_and: {
    out = in1 & in2;
    break;
  }
  case i_or: {
    out = in1 | in2;
    break;
  }
  case i_xor: {
    out = in1 ^ in2;
    break;
  }
  case i_rl: {
    out = in1<<1;
    break;
  }
  case i_rr: {
    out = in2>>1;
    break;
  }
  // data transfer
  case i_mov: {
    out = in1;
    break;
  }
  // branching (out==0 -> don't branch)
  case i_call:
  case i_ret:
  case i_jmp:
  case i_sjmp: {
    out = 1;
    break;
  }
  case i_jz: {
    out = (A==0);
    break;
  }
  case i_jnz: {
    out = (A!=0);
    break;
  }
  case i_cjne: {
    out = (in1!=in2);
    break;
  }
  case i_djnz: {
    out=in1-1; // decrement reg/direct and jump if != 0
    break;
  }
  default: {
    break;
  }
  }


  // write back --------------------------------------------------------
  write_back(&(i->dst),out);

  // compute next address ----------------------------------------------
  switch(i->type) {
  case i_call: {
    stack_el *new_stack_el= (stack_el *) malloc(sizeof(stack_el));
    new_stack_el->up = my_stack;
    new_stack_el->address = in1;
    my_stack = new_stack_el;

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);
    
    break;
  }
  case i_ret: {
    stack_el *new_stack_el = my_stack->up;
    free(my_stack);
    my_stack = new_stack_el;
    if(my_stack!=NULL)
      my_stack->address += 1; // increment address after jump

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);
    exec_bus_cycle(OP_IDLE,0,0,&result);
    exec_bus_cycle(OP_IDLE,0,0,&result);
    break;
  }
  case i_jmp: {
    my_stack->address = in1;

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);
     
    break;
  }
  case i_sjmp:
  case i_jz:
  case i_jnz: {
    if(out!=0)
      my_stack->address += in1+1;
    else
      my_stack->address += 1;

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);
 
    break;
  }
  case i_cjne: {
    int in3 = fetch_operand(&i->dst); 
    if(out!=0)
      my_stack->address += in3+1;
    else
      my_stack->address += 1;

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);

    break;
  }
  case i_djnz: {
    if(out!=0)
      my_stack->address += in2+1;
    else
      my_stack->address += 1;

    /* wait additional cycles */
    int result;
    exec_bus_cycle(OP_IDLE,0,0,&result);

    break;
  }
  default: {
     my_stack->address += 1;
     break;
  }
  }
}



//---------------------------------------------------------------------
// cycle_model::init() 
//
//   initialize the stack
//   
//---------------------------------------------------------------------
void cycle_model::init() {

  cycles2execute = 0;
  cycle_count = 0;
  stretch_cycles = 0;
  
  // initialize stack
  my_stack = (stack_el *) malloc(sizeof(stack_el));
  my_stack->up = NULL;
  my_stack->address = 0;
}



//------------------------------------------------------------------------
// void cycle_mode::entry() 
//
// main loop: fetch instruction
//            decode opcode
//            execute instruction
//
//------------------------------------------------------------------------
void cycle_model::entry() {

  wait();

  mem_ale.write(0);
  mem_wr_n.write(1);
  mem_pswr_n.write(1);
  mem_rd_n.write(1);
  mem_psrd_n.write(1);
  p0_mem_reg_n.write(0);
  p0_addr_data_n.write(0);
  wait();

  while(true) {
    instr the_instr;
    // fetch instruction
    if(my_stack==NULL) {
      // printf("cycles count = %d\n",cycle_count);
      sc_stop();
      wait();
    } else {
      int opcode = fetch_instr(my_stack->address);
      
      // decode instruction
      decode(opcode, &the_instr);
      
      // execute
      execute(&the_instr);
    }
  }
}
