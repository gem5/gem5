
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
 * AssignStatementAST.C
 *
 * Description: See AssignStatementAST.h
 *
 * $Id: AssignStatementAST.C,v 3.2 2003/08/01 18:38:19 beckmann Exp $
 *
 */

#include "AssignStatementAST.hh"

AssignStatementAST::AssignStatementAST(ExprAST* lvalue_ptr, ExprAST* rvalue_ptr)
  : StatementAST()
{
  m_lvalue_ptr = lvalue_ptr;
  m_rvalue_ptr = rvalue_ptr;
}

AssignStatementAST::~AssignStatementAST()
{
  delete m_lvalue_ptr;
  delete m_rvalue_ptr;
}

void AssignStatementAST::generate(string& code, Type* return_type_ptr) const
{
  code += indent_str();
  Type* lvalue_type_ptr = m_lvalue_ptr->generate(code);
  code += " = ";
  Type* rvalue_type_ptr = m_rvalue_ptr->generate(code);
  code += ";\n";

  if (lvalue_type_ptr != rvalue_type_ptr) {
    // FIXME - beckmann
    // the following if statement is a hack to allow NetDest objects to be assigned to Sets
    // this allows for the previous NetworkMessage Destiantion 'Set class' to migrate to the
    // new NetworkMessage Destiantion 'NetDest class'
    if (lvalue_type_ptr->toString() != "NetDest" && rvalue_type_ptr->toString() != "Set") {
      error("Assignment type mismatch '" + lvalue_type_ptr->toString() + "' and '" + rvalue_type_ptr->toString() + "'");
    }
  }
}

void AssignStatementAST::print(ostream& out) const
{
  out << "[AssignStatementAST: " << *m_lvalue_ptr << " := " << *m_rvalue_ptr << "]";
}
