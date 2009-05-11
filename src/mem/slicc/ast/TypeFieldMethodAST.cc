
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
 * TypeFieldMethodAST.C
 *
 * Description: See TypeFieldMethodAST.h
 *
 * $Id: TypeFieldMethodAST.C,v 3.1 2003/07/10 18:08:07 milo Exp $
 *
 */

#include "TypeFieldMethodAST.hh"

TypeFieldMethodAST::TypeFieldMethodAST(TypeAST* return_type_ast_ptr,
                                       string* ident_ptr,
                                       Vector<TypeAST*>* type_ast_vec_ptr,
                                       PairListAST* pairs_ptr)
  : TypeFieldAST(pairs_ptr)
{
  m_return_type_ast_ptr = return_type_ast_ptr;
  m_ident_ptr = ident_ptr;
  m_type_ast_vec_ptr = type_ast_vec_ptr;
}

TypeFieldMethodAST::~TypeFieldMethodAST()
{
  delete m_return_type_ast_ptr;
  delete m_ident_ptr;

  int size = m_type_ast_vec_ptr->size();
  for(int i=0; i<size; i++) {
    delete (*m_type_ast_vec_ptr)[i];
  }
  delete m_type_ast_vec_ptr;
}

void TypeFieldMethodAST::generate(Type *type_ptr)
{
  // Lookup return type
  Type* return_type_ptr = m_return_type_ast_ptr->lookupType();

  // Lookup parameter types
  Vector<Type*> type_vec;
  int size = m_type_ast_vec_ptr->size();
  for(int i=0; i<size; i++) {
    Type* type_ptr = (*m_type_ast_vec_ptr)[i]->lookupType();
    type_vec.insertAtBottom(type_ptr);
  }

  // Add method
  if (!type_ptr->methodAdd(*m_ident_ptr, return_type_ptr, type_vec)) {  // Return false on error
    error("Duplicate method: " + type_ptr->toString() + ":" + *m_ident_ptr + "()");
  }
}
