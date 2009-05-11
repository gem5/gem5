
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
 * TypeFieldMemberAST.C
 *
 * Description: See TypeFieldMemberAST.h
 *
 * $Id: TypeFieldMemberAST.C,v 3.1 2003/03/27 22:58:54 xu Exp $
 *
 */

#include "TypeFieldMemberAST.hh"

TypeFieldMemberAST::TypeFieldMemberAST(TypeAST* type_ast_ptr,
                                       string* field_id_ptr,
                                       PairListAST* pairs_ptr,
                                       ExprAST* rvalue_ptr)
  : TypeFieldAST(pairs_ptr)
{
  m_type_ast_ptr = type_ast_ptr;
  m_field_id_ptr = field_id_ptr;
  m_rvalue_ptr = rvalue_ptr;
}

TypeFieldMemberAST::~TypeFieldMemberAST()
{
  delete m_type_ast_ptr;
  delete m_field_id_ptr;
  if(m_rvalue_ptr) delete m_rvalue_ptr;
}

void TypeFieldMemberAST::generate(Type *type_ptr)
{
  // Lookup type
  Type* field_type_ptr = m_type_ast_ptr->lookupType();

  // check type if this is a initialization
  string* init_code = NULL;
  if(m_rvalue_ptr) {
    init_code = new string();
    Type* rvalue_type_ptr = m_rvalue_ptr->generate(*init_code);
    if(field_type_ptr != rvalue_type_ptr) {
      error("Initialization type mismatch '" + field_type_ptr->toString() + "' and '" + rvalue_type_ptr->toString() + "'");
    }
  }

  // Add data member to the parent type
  if (!type_ptr->dataMemberAdd(*m_field_id_ptr, field_type_ptr, getPairs(),
                               init_code)) {
    error("Duplicate data member: " + type_ptr->toString() + ":" + *m_field_id_ptr);
  }
}

void TypeFieldMemberAST::print(ostream& out) const
{
  out << "[TypeFieldMember: " << *m_field_id_ptr << "]";
}
