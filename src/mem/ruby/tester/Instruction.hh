/*
 * Copyright (c) 1999 by Mark Hill and David Wood for the Wisconsin
 * Multifacet Project.  ALL RIGHTS RESERVED.
 *
 * ##HEADER##
 *
 * This software is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and the
 * inclusion of the above copyright notice.  This software or any
 * other copies thereof or any derivative works may not be provided or
 * otherwise made available to any other persons.  Title to and
 * ownership of the software is retained by Mark Hill and David Wood.
 * Any use of this software must include the above copyright notice.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  THE LICENSOR MAKES NO
 * WARRANTIES ABOUT ITS CORRECTNESS OR PERFORMANCE.
 * */

/*
 * $Id: Instruction.h 1.2 05/05/24 12:15:47-05:00 kmoore@balder.cs.wisc.edu $
 *
 * Description:
 *
 */

#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include "Address.hh"


enum Opcode {
  Opcode_BEGIN,
  Opcode_LD,
  Opcode_ST,
  Opcode_INC,
  Opcode_COMMIT,
  Opcode_DONE,
  Opcode_NUM_OPCODES
};

class Instruction {
 public:
  Instruction();
  Instruction(Opcode op, Address addr);

  void init(Opcode op, Address addr);
  Opcode getOpcode();
  Address getAddress();

 private:
  Opcode m_opcode;
  Address m_address;

};

#endif
