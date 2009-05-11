
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
 * OutPortDeclAST.C
 *
 * Description: See OutPortDeclAST.h
 *
 * $Id: OutPortDeclAST.C,v 3.3 2004/02/02 22:37:51 milo Exp $
 *
 */

#include "mem/slicc/ast/OutPortDeclAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

OutPortDeclAST::OutPortDeclAST(string* ident_ptr,
                               TypeAST* msg_type_ptr,
                               ExprAST* var_expr_ptr,
                               PairListAST* pairs_ptr)
  : DeclAST(pairs_ptr)
{
  m_ident_ptr = ident_ptr;
  m_msg_type_ptr = msg_type_ptr;
  m_var_expr_ptr = var_expr_ptr;
  m_queue_type_ptr = new TypeAST(new string("OutPort"));
}

OutPortDeclAST::~OutPortDeclAST()
{
  delete m_ident_ptr;
  delete m_msg_type_ptr;
  delete m_var_expr_ptr;
  delete m_queue_type_ptr;
}

void OutPortDeclAST::generate()
{
  string code;
  Type* queue_type_ptr = m_var_expr_ptr->generate(code);
  if (!queue_type_ptr->isOutPort()) {
    error("Outport queues must be of a type that has the 'outport' attribute.  The type '" +
          queue_type_ptr->toString() + "' does not have this attribute.");
  }

  Type* type_ptr = m_queue_type_ptr->lookupType();
  g_sym_table.newSym(new Var(*m_ident_ptr, getLocation(), type_ptr, code, getPairs()));
}


void OutPortDeclAST::print(ostream& out) const
{
  out << "[OutPortDecl: " << *m_ident_ptr << "]";
}
