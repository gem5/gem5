
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
 * ExprStatementAST.C
 *
 * Description: See ExprStatementAST.h
 *
 * $Id$
 *
 */

#include "ExprStatementAST.hh"

ExprStatementAST::ExprStatementAST(ExprAST* expr_ptr)
  : StatementAST()
{
  m_expr_ptr = expr_ptr;
}

ExprStatementAST::~ExprStatementAST()
{
  delete m_expr_ptr;
}

void ExprStatementAST::generate(string& code, Type* return_type_ptr) const
{
  code += indent_str();
  Type* actual_type_ptr = m_expr_ptr->generate(code);
  code += ";\n";

  // The return type must be void
  Type* expected_type_ptr = g_sym_table.getType("void");
  if (expected_type_ptr != actual_type_ptr) {
    m_expr_ptr->error("Non-void return must not be ignored, return type is '" + actual_type_ptr->toString() + "'");
  }
}

void ExprStatementAST::findResources(Map<Var*, string>& resource_list) const
{
  m_expr_ptr->findResources(resource_list);
}

void ExprStatementAST::print(ostream& out) const
{
  out << "[ExprStatementAST: " << *m_expr_ptr << "]";
}
