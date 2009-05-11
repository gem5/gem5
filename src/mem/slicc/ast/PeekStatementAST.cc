
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
 * PeekStatementAST.C
 *
 * Description: See PeekStatementAST.h
 *
 * $Id$
 *
 */

#include "PeekStatementAST.hh"
#include "SymbolTable.hh"
#include "StatementListAST.hh"
#include "TypeAST.hh"
#include "VarExprAST.hh"

PeekStatementAST::PeekStatementAST(VarExprAST* queue_name_ptr,
                                   TypeAST* type_ptr,
                                   StatementListAST* statementlist_ptr,
                                   string method)
  : StatementAST()
{
  m_queue_name_ptr = queue_name_ptr;
  m_type_ptr = type_ptr;
  m_statementlist_ptr = statementlist_ptr;
  m_method = method;
}

PeekStatementAST::~PeekStatementAST()
{
  delete m_queue_name_ptr;
  delete m_type_ptr;
  delete m_statementlist_ptr;
}

void PeekStatementAST::generate(string& code, Type* return_type_ptr) const
{
  code += indent_str() + "{\n";  // Start scope
  inc_indent();
  g_sym_table.pushFrame();

  Type* msg_type_ptr = m_type_ptr->lookupType();

  // Add new local var to symbol table
  g_sym_table.newSym(new Var("in_msg", getLocation(), msg_type_ptr, "(*in_msg_ptr)", getPairs()));

  // Check the queue type
  m_queue_name_ptr->assertType("InPort");

  // Declare the new "in_msg_ptr" variable
  code += indent_str() + "const " + msg_type_ptr->cIdent() + "* in_msg_ptr;\n";  // Declare message
  //  code += indent_str() + "in_msg_ptr = static_cast<const ";
  code += indent_str() + "in_msg_ptr = dynamic_cast<const ";
  code += msg_type_ptr->cIdent() + "*>(";
  code += "(" + m_queue_name_ptr->getVar()->getCode() + ")";
  code += ".";
  code += m_method;
  code += "());\n";

  code += indent_str() + "assert(in_msg_ptr != NULL);\n";        // Check the cast result

  if(CHECK_INVALID_RESOURCE_STALLS) {
    // Declare the "in_buffer_rank" variable
    code += indent_str() + "int in_buffer_rank = ";  // Declare message
    code += "(" + m_queue_name_ptr->getVar()->getCode() + ")";
    code += ".getPriority();\n";
  }

  m_statementlist_ptr->generate(code, return_type_ptr);                // The other statements
  dec_indent();
  g_sym_table.popFrame();
  code += indent_str() + "}\n";  // End scope
}

void PeekStatementAST::findResources(Map<Var*, string>& resource_list) const
{
  m_statementlist_ptr->findResources(resource_list);
}

void PeekStatementAST::print(ostream& out) const
{
  out << "[PeekStatementAST: " << m_method
      << " queue_name: " << *m_queue_name_ptr
      << " type: " << m_type_ptr->toString()
      << " " << *m_statementlist_ptr
      << "]";
}
