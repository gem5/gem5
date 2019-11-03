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

  cycle_model.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "common.h"

#define MEM_SIZE 65536
#define INT_SIZE 2048

/* types of operands */
enum op_type {
  o_null,
  o_acc, // accumulator
  o_reg, // registers Ri
  o_dir, // internal register direct
  o_ind, // internal register pointed to by R0 or R1
  o_ext, // external memory pointed to by R0 or R1
  o_rel, // 2complement offset byte
  o_cst, // 8bit constant
  o_lcst, // 16bit constant
  o_add, // 11bit destination address
  o_ladd // 16bit destination address
};


/* limited set of instruction types */
enum instr_type {
  // arithmetic operations
  i_add, // 0
  i_sub, // 1
  i_inc, // 2
  i_dec, // 3
  i_mul, // 4
  i_div, // 5
  // logic operations
  i_and, // 6
  i_or,  // 7
  i_xor, // 8
  i_clr, // 9
  i_cpl, // 10
  i_rl,  // 11
  i_rr,  // 12
  // data transfer
  i_mov, // 13
  // branching
  i_call,// 14
  i_ret, // 15
  i_jmp, // 16
  i_sjmp,// 17
  i_jz,  // 18
  i_jnz, // 19
  i_cjne,// 20
  i_djnz,// 21
  i_nop  // 22
};


/* cycle types for memory bus */
enum bus_cycle_type {
  OP_IDLE,
  OP_MEM_READ,
  OP_MEM_WRITE
};


/* ------------------------------------------------------------------------
 * struct operand
 * ----------------------------------------------------------------------*/
struct operand {
  op_type type;
  int     val; /* value encoded with the opcode (i.e. register number) */
};


/* ------------------------------------------------------------------------
 * struct instr  ( instruction )
 * ----------------------------------------------------------------------*/
struct instr {
  instr_type type; 
  int n_src;    /* number of source operands */
  operand src1; /* source operand 1 */
  operand src2; /* source operand 2 */
  operand dst;  /* destination operand */
  int cycle;    // number of cycles 
};


/* ------------------------------------------------------------------------
 * struct stack_el ( stack element )
 * ----------------------------------------------------------------------*/
struct stack_el {
  int address;
  stack_el *up;
};



/* ------------------------------------------------------------------------
 *  struct cycle_model: public sc_aproc
 *
 * ----------------------------------------------------------------------*/

SC_MODULE( cycle_model )
{
  SC_HAS_PROCESS( cycle_model );

  sc_in_clk clk;

  /* functions */
  void init();
  void parse_hex(char *name);
  void exec_bus_cycle(bus_cycle_type op, int addr, int data, int* result);
  bool request_address(int addr);
  int fetch_instr(int ad);
  int fetch_data(int ad);
  int write_data(int ad, int data);
  int fetch_operand(operand* op);
  int write_back(operand *, int);
  void execute(instr *i);
  void decode(int opcode, instr* i);
  

  /* memory bus */
  signal_bool_vector16&                 mem_addr;
  signal_bool_vector8&                  mem_data_out;
  const signal_bool_vector8&            mem_data_in;
  sc_signal<bool>&                      mem_wr_n;
  sc_signal<bool>&                      mem_rd_n;
  sc_signal<bool>&                      mem_pswr_n;
  sc_signal<bool>&                      mem_psrd_n;
  sc_signal<bool>&                      mem_ale;
  const sc_signal<bool>&                mem_ea_n;

  sc_signal<bool>&                      p0_mem_reg_n;
  sc_signal<bool>&                      p0_addr_data_n;
  sc_signal<bool>&                      p2_mem_reg_n;

  /* internal variables */
  /* memories registers and stack */
  int instr_mem[MEM_SIZE]; /* instruction memory */
  int ext_mem[MEM_SIZE];   /* 'external' data memory */
  int int_mem[INT_SIZE];   /* internal data memory */ 
  int A;                   /* accumulator */
  int R[8];                /* registers */
  stack_el *my_stack;      /* stack */
  /* others */
  int cycles2execute;      /* number of cycles to execute */
  int cycle_count;         /* total number of cycles executed */
  int stretch_cycles;      /* memory stretch cycles */



  /* Constructor */ 
  cycle_model(sc_module_name                   NAME,
	      const sc_signal_in_if<bool>&     CLK,
	      char*                            hex_file_name,

	      signal_bool_vector16&            MEM_ADDR,
	      signal_bool_vector8&             MEM_DATA_OUT,
	      const signal_bool_vector8&       MEM_DATA_IN,
	      sc_signal<bool>&                 MEM_WR_N,
	      sc_signal<bool>&                 MEM_RD_N,
	      sc_signal<bool>&                 MEM_PSWR_N,
	      sc_signal<bool>&                 MEM_PSRD_N,
	      sc_signal<bool>&                 MEM_ALE,
	      const sc_signal<bool>&           MEM_EA_N,
	      
	      sc_signal<bool>&                 P0_MEM_REG_N,
	      sc_signal<bool>&                 P0_ADDR_DATA_N,
	      sc_signal<bool>&                 P2_MEM_REG_N
	      )
    : 
      
      mem_addr(MEM_ADDR), 
      mem_data_out(MEM_DATA_OUT),
      mem_data_in(MEM_DATA_IN),
      mem_wr_n(MEM_WR_N),
      mem_rd_n(MEM_RD_N),
      mem_pswr_n(MEM_PSWR_N),
      mem_psrd_n(MEM_PSRD_N),
      mem_ale(MEM_ALE),      
      mem_ea_n(MEM_EA_N),
      p0_mem_reg_n(P0_MEM_REG_N),
      p0_addr_data_n(P0_ADDR_DATA_N),
      p2_mem_reg_n(P2_MEM_REG_N)
    {
      clk(CLK);
	  SC_THREAD( entry );
      sensitive << clk;
      
      parse_hex(hex_file_name);
      init();
    }
  
  /* Process functionality in member function below */
  void entry();
};
