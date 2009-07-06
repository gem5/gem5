
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
 * TypeDeclAST.C
 *
 * Description: See TypeDeclAST.hh
 *
 * $Id: TypeDeclAST.C,v 3.1 2003/03/22 15:15:17 xu Exp $
 *
 */

#include "mem/slicc/ast/TypeDeclAST.hh"
#include "mem/slicc/main.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

TypeDeclAST::TypeDeclAST(TypeAST* type_ast_ptr,
                         PairListAST* pairs_ptr,
                         Vector<TypeFieldAST*>* field_vec_ptr)
  : DeclAST(pairs_ptr)
{
  m_type_ast_ptr = type_ast_ptr;
  m_field_vec_ptr = field_vec_ptr;
}

TypeDeclAST::~TypeDeclAST()
{
  delete m_type_ast_ptr;
  if (m_field_vec_ptr != NULL) {
    int size = m_field_vec_ptr->size();
    for(int i=0; i<size; i++) {
      delete (*m_field_vec_ptr)[i];
    }
    delete m_field_vec_ptr;
  }
}

void TypeDeclAST::generate()
{
  string machine_name;
  string id = m_type_ast_ptr->toString();

  // Make the new type
  Type* new_type_ptr = new Type(id, getLocation(), getPairs(),
                                g_sym_table.getStateMachine());
  g_sym_table.newSym(new_type_ptr);

  // Add all of the fields of the type to it
  if (m_field_vec_ptr != NULL) {
    int size = m_field_vec_ptr->size();
    for(int i=0; i<size; i++) {
      (*m_field_vec_ptr)[i]->generate(new_type_ptr);
    }
  }
}

void TypeDeclAST::print(ostream& out) const
{
  out << "[TypeDecl: " << m_type_ast_ptr->toString() << "]";
}
