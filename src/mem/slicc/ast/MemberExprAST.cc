
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
 * FieldExprAST.C
 *
 * Description: See FieldExprAST.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/MemberExprAST.hh"

MemberExprAST::MemberExprAST(ExprAST* expr_ast_ptr, string* field_ptr)
  : ExprAST()
{
  m_expr_ast_ptr = expr_ast_ptr;
  m_field_ptr = field_ptr;
}

MemberExprAST::~MemberExprAST()
{
  delete m_expr_ast_ptr;
  delete m_field_ptr;
}

Type* MemberExprAST::generate(string& code) const
{
  code += "(";
  Type* type_ptr = m_expr_ast_ptr->generate(code);
  code += ").m_" + (*m_field_ptr);

  // Verify that this is a valid field name for this type
  if (!type_ptr->dataMemberExist(*m_field_ptr)) {
    error("Invalid object field: Type '" + type_ptr->toString() + "' does not have data member " + *m_field_ptr);
  }

  // Return the type of the field
  return type_ptr->dataMemberType(*m_field_ptr);
}

void MemberExprAST::print(ostream& out) const
{
  out << "[MemberExprAST: " << *m_expr_ast_ptr << "." << *m_field_ptr << "]";
}
