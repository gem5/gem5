
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
 * ObjDeclAST.cc
 *
 * Description: See ObjDeclAST.hh
 *
 * $Id: ObjDeclAST.cc,v 3.13 2004/06/24 15:56:14 beckmann Exp $
 *
 */

#include "mem/slicc/ast/ObjDeclAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/main.hh"

ObjDeclAST::ObjDeclAST(TypeAST* type_ptr,
                       string* ident_ptr,
                       PairListAST* pairs_ptr)
  : DeclAST(pairs_ptr)
{
  m_type_ptr = type_ptr;
  m_ident_ptr = ident_ptr;
}

ObjDeclAST::~ObjDeclAST()
{
  delete m_type_ptr;
  delete m_ident_ptr;
}

void ObjDeclAST::generate()
{

  bool machineComponentSym = false;

  getPairs().add("chip_object", "yes");

  string c_code;


  if (getPairs().exist("hack")) {
    warning("'hack=' is now deprecated");
  }

  if (getPairs().exist("network")) {
    if (!getPairs().exist("virtual_network")) {
      error("Network queues require a 'virtual_network' attribute.");
    }
  }

  Type* type_ptr = m_type_ptr->lookupType();
  if (type_ptr->isBuffer()) {
    if (!getPairs().exist("ordered")) {
      error("Buffer object declarations require an 'ordered' attribute.");
    }
  }

  if (getPairs().exist("ordered")) {
    string value = getPairs().lookup("ordered");
    if (value != "true" && value != "false") {
      error("The 'ordered' attribute must be 'true' or 'false'.");
    }
  }

  if (getPairs().exist("random")) {
    string value = getPairs().lookup("random");
    if (value != "true" && value != "false") {
      error("The 'random' attribute must be 'true' or 'false'.");
    }
  }

  string machine;
  if (g_sym_table.getStateMachine() != NULL) {
    machine = g_sym_table.getStateMachine()->getIdent() + "_";
  }

  // FIXME : should all use accessors here to avoid public member variables
  if (*m_ident_ptr == "id") {
    c_code = "m_chip_ptr->getID()";
  } else if (*m_ident_ptr == "version") {
    c_code = "m_version";
  } else if (*m_ident_ptr == "machineID") {
    c_code = "m_machineID";
  } else if (*m_ident_ptr == "sequencer") {
    c_code = "*(dynamic_cast<"+m_type_ptr->toString()+"*>(m_chip_ptr->getSequencer(m_version)))";
    machineComponentSym = true;
  } /*else if (*m_ident_ptr == "xfdr_record_mgr") {
    c_code = "*(dynamic_cast<"+m_type_ptr->toString()+"*>(m_chip_ptr->getXfdrManager(m_version)))";
    machineComponentSym = true;
    } */else if (// getPairs().exist("network") || (m_type_ptr->lookupType()->existPair("cache"))
//              || (m_type_ptr->lookupType()->existPair("tbe")) ||
//              (m_type_ptr->lookupType()->existPair("newtbe")) ||
//              (m_type_ptr->lookupType()->existPair("timer")) ||
//              (m_type_ptr->lookupType()->existPair("dir")) ||
//              (m_type_ptr->lookupType()->existPair("persistent")) ||
//              (m_type_ptr->lookupType()->existPair("filter")) ||
//              (getPairs().exist("trigger_queue"))
             getPairs().exist("no_vector")) {
    c_code = "(*(m_chip_ptr->m_" + machine + *m_ident_ptr + "_ptr))";
    machineComponentSym = true;
  } else {
    c_code = "(*(m_chip_ptr->m_" + machine + *m_ident_ptr + "_vec[m_version]))";
    machineComponentSym = true;
  }

  Var* v = new Var(*m_ident_ptr, getLocation(), type_ptr, c_code,
                             getPairs(), g_sym_table.getStateMachine());

  g_sym_table.newSym(v);

  // used to cheat-- that is, access components in other machines
  if (machineComponentSym) {
    g_sym_table.newMachComponentSym(v);
  }

}

void ObjDeclAST::print(ostream& out) const
{
  out << "[ObjDecl: " << *m_ident_ptr << "]";
}
