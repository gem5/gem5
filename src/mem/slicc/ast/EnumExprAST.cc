
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
 * EnumExprAST.C
 *
 * Description: See EnumExprAST.h
 *
 * $Id: EnumExprAST.C,v 3.1 2003/07/10 18:08:06 milo Exp $
 *
 */

#include "EnumExprAST.hh"

EnumExprAST::EnumExprAST(TypeAST* type_ast_ptr,
                         string* value_ptr)
  : ExprAST()
{
  assert(value_ptr != NULL);
  assert(type_ast_ptr != NULL);
  m_type_ast_ptr = type_ast_ptr;
  m_value_ptr = value_ptr;
}

EnumExprAST::~EnumExprAST()
{
  delete m_type_ast_ptr;
  delete m_value_ptr;
}

Type* EnumExprAST::generate(string& code) const
{
  Type* type_ptr = m_type_ast_ptr->lookupType();
  code += type_ptr->cIdent() + "_" + (*m_value_ptr);

  // Make sure the enumeration value exists
  if (!type_ptr->enumExist(*m_value_ptr)) {
    error("Type '" + m_type_ast_ptr->toString() + "' does not have enumeration '" + *m_value_ptr + "'");
  }

  // Return the proper type
  return type_ptr;
}

void EnumExprAST::print(ostream& out) const
{
  string str;
  str += m_type_ast_ptr->toString()+":"+(*m_value_ptr);
  out << "[EnumExpr: " << str << "]";
}
