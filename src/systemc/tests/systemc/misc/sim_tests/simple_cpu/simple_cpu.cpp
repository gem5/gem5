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

  simple_cpu.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

#define READ 0
#define WRITE 1

SC_MODULE( exec_decode )
{
  SC_HAS_PROCESS( exec_decode );

  sc_in<unsigned>  instruction;
  sc_signal<unsigned>& program_counter;

  unsigned pc; // Program counter
  unsigned cpu_reg[32]; // Cpu registers

  unsigned *data_mem; // The data memory

  exec_decode( sc_module_name NAME,
	       sc_signal<unsigned>& INSTRUCTION,
	       sc_signal<unsigned>& PROGRAM_COUNTER )
    : program_counter(PROGRAM_COUNTER)
  {
    instruction(INSTRUCTION);
	SC_METHOD( entry );
    // sensitive only to the clock
    sensitive << instruction;

    pc = 0x000000; // Power up reset value
    for (int i =0; i<32; i++) cpu_reg[i] = 0;

    // Initialize the data memory from file datamem
    FILE *fp = fopen("simple_cpu/datamem", "r");
    if (fp == (FILE *) 0) return; // No data mem file to read
    // First field in this file is the size of data memory desired
    int size;
    fscanf(fp, "%d", &size);
    data_mem = new unsigned[size];
    if (data_mem == (unsigned *) 0) {
      printf("Not enough memory left\n");
      return;
    }
    unsigned mem_word;
    size = 0;
    while (fscanf(fp, "%x", &mem_word) != EOF) {
      data_mem[size++] = mem_word;
    }

    // Start off simulation by writing program_counter
    program_counter.write(pc);
  }

  // Functionality
  void entry();
};

void 
exec_decode::entry()
{
  unsigned instr;
  unsigned opcode;
  unsigned regnum1, regnum2, regnum3;
  unsigned addr;

  int i;

  instr = instruction.read();
  opcode = (instr & 0xe0000000) >> 29; // Extract opcode
  switch(opcode) {
  case 0x0: // Halt
    printf("CPU Halted\n");
    printf("\tPC = 0x%x\n", pc);
    for (i = 0; i < 32; i++)
      printf("\tR[%d] = %x\n", i, cpu_reg[i]);
    // Don't write pc and execution will stop
    break;
  case 0x1: // Store
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    addr = (instr & 0x00ffffff); //  Extract address
    printf("Store: Memory[0x%x] = R[%d]\n", addr, regnum1);
    data_mem[addr] = cpu_reg[regnum1];
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x2: // Load
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    addr = (instr & 0x00ffffff); //  Extract address
    printf("Load: R[%d] = Memory[0x%x]\n", regnum1, addr);
    cpu_reg[regnum1] = data_mem[addr];
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x3: // Add
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    regnum2 = (instr & 0x00f80000) >> 19; // Extract register number
    regnum3 = (instr & 0x0007c000) >> 14; // Extract register number
    printf("R[%d] = R[%d] + R[%d]\n", regnum3, regnum1, regnum2);
    cpu_reg[regnum3] = cpu_reg[regnum1] + cpu_reg[regnum2];
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x4: // Subtract
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    regnum2 = (instr & 0x00f80000) >> 19; // Extract register number
    regnum3 = (instr & 0x0007c000) >> 14; // Extract register number
    printf("R[%d] = R[%d] - R[%d]\n", regnum3, regnum1, regnum2);
    cpu_reg[regnum3] = cpu_reg[regnum1] - cpu_reg[regnum2];
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x5: // Multiply
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    regnum2 = (instr & 0x00f80000) >> 19; // Extract register number
    regnum3 = (instr & 0x0007c000) >> 14; // Extract register number
    printf("R[%d] = R[%d] * R[%d]\n", regnum3, regnum1, regnum2);
    cpu_reg[regnum3] = cpu_reg[regnum1] * cpu_reg[regnum2];
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x6: // Divide
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    regnum2 = (instr & 0x00f80000) >> 19; // Extract register number
    regnum3 = (instr & 0x0007c000) >> 14; // Extract register number
    printf("R[%d] = R[%d] / R[%d]\n", regnum3, regnum1, regnum2);
    if (cpu_reg[regnum2] == 0) {
      printf("Division exception - divide by zero\n");
    }
    else {
      cpu_reg[regnum3] = cpu_reg[regnum1] / cpu_reg[regnum2];
    }
    pc = pc + 1;
    program_counter.write(pc);
    break;
  case 0x7: // JNZ
    regnum1 = (instr & 0x1f000000) >> 24; // Extract register number
    addr = (instr & 0x00ffffff); //  Extract address
    printf("JNZ R[%d] 0x%x\n", regnum1, addr);
    if (cpu_reg[regnum1] == 0x0) 
      pc = pc + 1;
    else
      pc = addr;
    program_counter.write(pc);
    break;
  default: // Bad opcode
    printf("Bad opcode 0x%x\n", opcode);
    pc = pc + 1;
    program_counter.write(pc);
    break; 
  }
}


SC_MODULE( fetch )
{
  SC_HAS_PROCESS( fetch );

  sc_in<unsigned> program_counter;
  sc_signal<unsigned>& instruction;

  unsigned *prog_mem; // The program memory

  fetch( sc_module_name NAME,
	 sc_signal<unsigned>& PROGRAM_COUNTER,
	 sc_signal<unsigned>& INSTRUCTION )
    : instruction(INSTRUCTION)
  {
    program_counter(PROGRAM_COUNTER);
    SC_METHOD( entry );
    sensitive << program_counter;

    // Initialize the program memory from file progmem
    FILE *fp = fopen("simple_cpu/progmem", "r");
    if (fp == (FILE *) 0) return; // No prog mem file to read
    // First field in this file is the size of program memory desired
    int size;
    fscanf(fp, "%d", &size);
    prog_mem = new unsigned[size];
    if (prog_mem == (unsigned *) 0) {
      printf("Not enough memory left\n");
      return;
    }
    unsigned mem_word;
    size = 0;
    while (fscanf(fp, "%x", &mem_word) != EOF) {
      prog_mem[size++] = mem_word;
    }
    instruction.write(0);
  }

  // Functionality
  void entry();
};

void fetch::entry()
{
  unsigned pc, instr;
  pc = program_counter.read();
  instr = prog_mem[pc];
  instruction.write(instr);
}

int 
sc_main(int ac, char *av[])
{
  sc_signal<unsigned> pc;
  sc_signal<unsigned> instr;

  exec_decode ED("ED", instr, pc);
  fetch F("F", pc, instr);

  // instead of a testbench routine, we include the testbench here
  sc_start(1, SC_NS);
  sc_start( 10, SC_NS );

  fflush( stdout );

  return 0;
}
