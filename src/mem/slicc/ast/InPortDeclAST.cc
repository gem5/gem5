
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
 * InPortDeclAST.C
 *
 * Description: See InPortDeclAST.h
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/InPortDeclAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/symbols/Var.hh"

InPortDeclAST::InPortDeclAST(string* ident_ptr,
                             TypeAST* msg_type_ptr,
                             ExprAST* var_expr_ptr,
                             PairListAST* pairs_ptr,
                             StatementListAST* statement_list_ptr)
  : DeclAST(pairs_ptr)
{
  m_ident_ptr = ident_ptr;
  m_msg_type_ptr = msg_type_ptr;
  m_var_expr_ptr = var_expr_ptr;
  m_statement_list_ptr = statement_list_ptr;
  m_queue_type_ptr = new TypeAST(new string("InPort"));
}

InPortDeclAST::~InPortDeclAST()
{
  delete m_ident_ptr;
  delete m_msg_type_ptr;
  delete m_var_expr_ptr;
  delete m_statement_list_ptr;
  delete m_queue_type_ptr;
}

void InPortDeclAST::generate()
{
  string code;
  Type* queue_type_ptr = m_var_expr_ptr->generate(code);
  if (!queue_type_ptr->isInPort()) {
    error("Inport queues must be of a type that has the 'inport' attribute.  The type '" +
          queue_type_ptr->toString() + "' does not have this attribute.");
  }

  Type* type_ptr = m_queue_type_ptr->lookupType();
  Var* in_port_ptr = new Var(*m_ident_ptr, getLocation(), type_ptr, code, getPairs());
  g_sym_table.newSym(in_port_ptr);

  g_sym_table.pushFrame();
  Vector<Type*> param_type_vec;

  // Check for Event
  type_ptr = g_sym_table.getType("Event");
  if (type_ptr == NULL) {
    error("in_port declarations require 'Event' enumeration to be defined");
  }
  param_type_vec.insertAtBottom(type_ptr);

  // Check for Address
  type_ptr = g_sym_table.getType("Address");
  if (type_ptr == NULL) {
    error("in_port declarations require 'Address' type to be defined");
  }
  param_type_vec.insertAtBottom(type_ptr);

  // Add the trigger method - FIXME, this is a bit dirty
  Map<string, string> pairs;
  pairs.add("external", "yes");
  Vector<string> string_vec;
  g_sym_table.newSym(new Func("trigger", getLocation(), g_sym_table.getType("void"), param_type_vec, string_vec, string(""), pairs, NULL));

  // Check for Event2
  type_ptr = g_sym_table.getType("Event");
  if (type_ptr == NULL) {
    error("in_port declarations require 'Event' enumeration to be defined");
  }
  param_type_vec.insertAtBottom(type_ptr);

  // Check for Address2
  type_ptr = g_sym_table.getType("Address");
  if (type_ptr == NULL) {
    error("in_port declarations require 'Address' type to be defined");
  }
  param_type_vec.insertAtBottom(type_ptr);

  // Add the doubleTrigger method - this hack supports tiggering two simulateous events
  // The key is that the second transistion cannot fail because the first event cannot be undone
  // therefore you must do some checks before calling double trigger to ensure that won't happen
  g_sym_table.newSym(new Func("doubleTrigger", getLocation(), g_sym_table.getType("void"), param_type_vec, string_vec, string(""), pairs, NULL));

  // Add the continueProcessing method - this hack supports messages that don't trigger events
  Vector<Type*> empty_param_type_vec;
  Vector<string> empty_string_vec;
  g_sym_table.newSym(new Func("continueProcessing", getLocation(), g_sym_table.getType("void"), empty_param_type_vec, empty_string_vec, string(""), pairs, NULL));

  if (m_statement_list_ptr != NULL) {
    inc_indent();
    inc_indent();
    string code;
    m_statement_list_ptr->generate(code, NULL);
    in_port_ptr->addPair("c_code_in_port", code);
    dec_indent();
    dec_indent();
  }
  g_sym_table.popFrame();

  // Add port to state machine
  StateMachine* machine_ptr = g_sym_table.getStateMachine();
  if (machine_ptr == NULL) {
    error("InPort declaration not part of a machine.");
  }
  machine_ptr->addInPort(in_port_ptr);
}


void InPortDeclAST::print(ostream& out) const
{
  out << "[InPortDecl: " << *m_ident_ptr << "]";
}
