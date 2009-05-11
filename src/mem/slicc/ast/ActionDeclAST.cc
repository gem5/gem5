
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
 * ActionDeclAST.C
 *
 * Description: See ActionDeclAST.h
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/ActionDeclAST.hh"
#include "mem/slicc/symbols/Action.hh"

ActionDeclAST::ActionDeclAST(string* ident_ptr,
                             PairListAST* pairs_ptr,
                             StatementListAST* statement_list_ptr)
  : DeclAST(pairs_ptr)
{
  m_ident_ptr = ident_ptr;
  m_statement_list_ptr = statement_list_ptr;
}

ActionDeclAST::~ActionDeclAST()
{
  delete m_ident_ptr;
  delete m_statement_list_ptr;
}

void ActionDeclAST::generate()
{
  Map<Var*, string> resource_list;
  if (m_statement_list_ptr != NULL) {
    string code;

    // Add new local vars
    g_sym_table.pushFrame();

    Type* type_ptr = g_sym_table.getType("Address");

    if (type_ptr == NULL) {
      error("Type 'Address' not declared.");
    }

    g_sym_table.newSym(new Var("address", getLocation(), type_ptr, "addr", getPairs()));

    // Don't allows returns in actions
    m_statement_list_ptr->generate(code, NULL);

    getPairs().add("c_code", code);

    m_statement_list_ptr->findResources(resource_list);

    g_sym_table.popFrame();
  }

  StateMachine* machine_ptr = g_sym_table.getStateMachine();
  if (machine_ptr == NULL) {
    error("Action declaration not part of a machine.");
  } else {
    machine_ptr->addAction(new Action(*m_ident_ptr, resource_list, getLocation(), getPairs()));
  }

}

void ActionDeclAST::print(ostream& out) const
{
  out << "[ActionDecl: " << *m_ident_ptr << "]";
}
