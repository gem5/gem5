
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
 * FuncDeclAST.C
 *
 * Description: See FuncDeclAST.hh
 *
 * $Id: FuncDeclAST.C,v 3.4 2003/08/22 18:19:34 beckmann Exp $
 *
 */

#include "mem/slicc/ast/FuncDeclAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/main.hh"

FuncDeclAST::FuncDeclAST(TypeAST* return_type_ast_ptr,
                         string* ident_ptr,
                         Vector<FormalParamAST*>* formal_vec_ptr,
                         PairListAST* pairs_ptr,
                         StatementListAST* statement_list_ptr)
  : DeclAST(pairs_ptr)
{
  m_return_type_ast_ptr = return_type_ast_ptr;
  m_ident_ptr = ident_ptr;
  m_formal_vec_ptr = formal_vec_ptr;
  m_statement_list_ptr = statement_list_ptr;
}

FuncDeclAST::~FuncDeclAST()
{
  delete m_return_type_ast_ptr;
  delete m_ident_ptr;

  int size = m_formal_vec_ptr->size();
  for(int i=0; i<size; i++) {
    delete (*m_formal_vec_ptr)[i];
  }
  delete m_formal_vec_ptr;
  delete m_statement_list_ptr;
}

void FuncDeclAST::generate()
{
  Vector<Type*> type_vec;
  Vector<string> param_vec;
  Type* void_type_ptr = g_sym_table.getType("void");

  // Generate definition code
  g_sym_table.pushFrame();

  // Lookup return type
  Type* return_type_ptr = m_return_type_ast_ptr->lookupType();

  // Generate function header
  int size = m_formal_vec_ptr->size();
  for(int i=0; i<size; i++) {
    // Lookup parameter types
    string ident;
    Type* type_ptr = (*m_formal_vec_ptr)[i]->generate(ident);
    type_vec.insertAtBottom(type_ptr);
    param_vec.insertAtBottom(ident);
  }

  string body;
  if (m_statement_list_ptr == NULL) {
    getPairs().add("external", "yes");
  } else {
    m_statement_list_ptr->generate(body, return_type_ptr);
  }
  g_sym_table.popFrame();

  StateMachine* machine_ptr = g_sym_table.getStateMachine();
  if (machine_ptr != NULL) {
    machine_ptr->addFunc(new Func(*m_ident_ptr, getLocation(), return_type_ptr, type_vec, param_vec, body, getPairs(), machine_ptr));
  } else {
    g_sym_table.newSym(new Func(*m_ident_ptr, getLocation(), return_type_ptr, type_vec, param_vec, body, getPairs(), machine_ptr));
  }

}

void FuncDeclAST::print(ostream& out) const
{
  out << "[FuncDecl: " << *m_ident_ptr << "]";
}
