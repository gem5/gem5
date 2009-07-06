
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
 * $Id$
 *
 */

#include "mem/slicc/ast/EnqueueStatementAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/ast/VarExprAST.hh"
#include "mem/slicc/ast/PairListAST.hh"
#include "mem/gems_common/util.hh"

EnqueueStatementAST::EnqueueStatementAST(VarExprAST* queue_name_ptr,
                                         TypeAST* type_name_ptr,
                                         PairListAST* pairs_ptr,
                                         StatementListAST* statement_list_ast_ptr)
  : StatementAST(pairs_ptr->getPairs())
{
  m_queue_name_ptr = queue_name_ptr;
  m_type_name_ptr = type_name_ptr;
  m_statement_list_ast_ptr = statement_list_ast_ptr;
}

EnqueueStatementAST::~EnqueueStatementAST()
{
  delete m_queue_name_ptr;
  delete m_type_name_ptr;
  delete m_statement_list_ast_ptr;
}

void EnqueueStatementAST::generate(string& code, Type* return_type_ptr) const
{
  code += indent_str() + "{\n";  // Start scope
  inc_indent();
  g_sym_table.pushFrame();

  Type* msg_type_ptr = m_type_name_ptr->lookupType();

  // Add new local var to symbol table
  g_sym_table.newSym(new Var("out_msg", getLocation(), msg_type_ptr, "out_msg", getPairs()));

  code += indent_str() + msg_type_ptr->cIdent() + " out_msg;\n";  // Declare message
  m_statement_list_ast_ptr->generate(code, NULL);                // The other statements

  code += indent_str();

  m_queue_name_ptr->assertType("OutPort");
  code += "(" + m_queue_name_ptr->getVar()->getCode() + ")";
  code += ".enqueue(out_msg";

  if (getPairs().exist("latency")) {
    code += ", RubyConfig::get" + getPairs().lookup("latency") + "()";
  }

  code += ");\n";

  dec_indent();
  g_sym_table.popFrame();
  code += indent_str() + "}\n";  // End scope
}

void EnqueueStatementAST::findResources(Map<Var*, string>& resource_list) const
{
  Var* var_ptr = m_queue_name_ptr->getVar();
  int res_count = 0;
  if (resource_list.exist(var_ptr)) {
    res_count = atoi((resource_list.lookup(var_ptr)).c_str());
  }
  resource_list.add(var_ptr, int_to_string(res_count+1));
}

void EnqueueStatementAST::print(ostream& out) const
{
  out << "[EnqueueStatementAst: " << *m_queue_name_ptr << " "
      << m_type_name_ptr->toString() << " " << *m_statement_list_ast_ptr << "]";
}
