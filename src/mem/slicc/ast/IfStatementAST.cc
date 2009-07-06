
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
 * IfStatementAST.C
 *
 * Description: See IfStatementAST.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/IfStatementAST.hh"

IfStatementAST::IfStatementAST(ExprAST* cond_ptr,
                               StatementListAST* then_ptr,
                               StatementListAST* else_ptr)
  : StatementAST()
{
  assert(cond_ptr != NULL);
  assert(then_ptr != NULL);
  m_cond_ptr = cond_ptr;
  m_then_ptr = then_ptr;
  m_else_ptr = else_ptr;
}

IfStatementAST::~IfStatementAST()
{
  delete m_cond_ptr;
  delete m_then_ptr;
  delete m_else_ptr;
}


void IfStatementAST::generate(string& code, Type* return_type_ptr) const
{
  Type* type_ptr;

  // Conditional
  code += indent_str() + "if (";
  type_ptr = m_cond_ptr->generate(code);
  if (type_ptr != g_sym_table.getType("bool")) {
    m_cond_ptr->error("Condition of if statement must be boolean, type was '" + type_ptr->toString() + "'");
  }
  code += ") {\n";
  // Then part
  inc_indent();
  m_then_ptr->generate(code, return_type_ptr);
  dec_indent();
  // Else part
  if (m_else_ptr != NULL) {
    code += indent_str() + "} else {\n";
    inc_indent();
    m_else_ptr->generate(code, return_type_ptr);
    dec_indent();
  }
  code += indent_str() + "}\n";  // End scope
}

void IfStatementAST::findResources(Map<Var*, string>& resource_list) const
{
  // Take a worse case look at both paths
  m_then_ptr->findResources(resource_list);
  if (m_else_ptr != NULL) {
    m_else_ptr->findResources(resource_list);
  }
}

void IfStatementAST::print(ostream& out) const
{
  out << "[IfStatement: " << *m_cond_ptr << *m_then_ptr << *m_else_ptr << "]";
}
