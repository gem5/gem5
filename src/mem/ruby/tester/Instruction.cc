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
 * $Id: Instruction.C 1.2 05/08/26 00:54:48-05:00 xu@s0-32.cs.wisc.edu $
 *
 * Description:
 *
 */

#include "mem/ruby/tester/Instruction.hh"

Instruction::Instruction(){
  m_opcode = Opcode_NUM_OPCODES;
  m_address = Address(physical_address_t(0));
}

Instruction::Instruction(Opcode op, Address addr){
  m_opcode = op;
  m_address = addr;
  assert(addr.getAddress() == 0);
}

void Instruction::init(Opcode op, Address addr){
  m_opcode = op;
  m_address = addr;
  //cout << "Instruction(" << op << ", " << m_address << ")" << endl;
}

Opcode Instruction::getOpcode(){
  return m_opcode;
}

Address Instruction::getAddress(){
  return m_address;
}
