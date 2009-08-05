
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * MachineAST.cc
 *
 * Description: See MachineAST.hh
 *
 * $Id: MachineAST.cc,v 3.1 2003/03/17 01:54:25 xu Exp $
 *
 */

#include "mem/slicc/ast/MachineAST.hh"
#include "mem/slicc/ast/FormalParamAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

MachineAST::MachineAST(string* ident_ptr,
                       PairListAST* pairs_ptr,
                       Vector<FormalParamAST*>* config_parameters,
                       DeclListAST* decl_list_ptr)

  : DeclAST(pairs_ptr)
{
  m_ident_ptr = ident_ptr;
  m_pairs_ptr = pairs_ptr;
  m_config_parameters = config_parameters;
  m_decl_list_ptr = decl_list_ptr;
}

MachineAST::~MachineAST()
{
  delete m_ident_ptr;
  delete m_pairs_ptr;
  delete m_decl_list_ptr;
}

void MachineAST::generate()
{
  StateMachine* machine_ptr;

  // Make a new frame
  g_sym_table.pushFrame();

  // Create a new machine
  machine_ptr = new StateMachine(*m_ident_ptr, getLocation(), getPairs(), m_config_parameters);
  g_sym_table.newCurrentMachine(machine_ptr);

  // Generate code for all the internal decls
  m_decl_list_ptr->generate();

  // Build the transition table
  machine_ptr->buildTable();

  // Pop the frame
  g_sym_table.popFrame();
}

void MachineAST::findMachines()
{
  // Add to MachineType enumeration
  Type* type_ptr = g_sym_table.getType("MachineType");
  if (!type_ptr->enumAdd(*m_ident_ptr, m_pairs_ptr->getPairs())) {
    error("Duplicate machine name: " + type_ptr->toString() + ":" + *m_ident_ptr);
  }

  // Generate code for all the internal decls
  m_decl_list_ptr->findMachines();
}

void MachineAST::print(ostream& out) const
{
  out << "[Machine: " << *m_ident_ptr << "]";
}
