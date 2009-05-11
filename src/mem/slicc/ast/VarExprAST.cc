
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
 * VarExprAST.C
 *
 * Description: See VarExprAST.h
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/VarExprAST.hh"
#include "mem/slicc/ast/StatementAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

VarExprAST::~VarExprAST()
{
  delete m_var_ptr;
}


Var* VarExprAST::getVar() const
{
  string var = *m_var_ptr;
  Var* var_ptr = g_sym_table.getVar(var);
  if (var_ptr == NULL) {
    error("Unrecognized variable: " + var);
  }
  return var_ptr;
}

void VarExprAST::assertType(string type_ident) const
{
  Type* expected_type_ptr = g_sym_table.getType(type_ident);
  if (expected_type_ptr == NULL) {
    error("There must be a type '" + type_ident + "' declared in this scope");
  }

  if (getVar()->getType() != expected_type_ptr) {
    error("Incorrect type: '" + getVar()->getIdent() + "' is expected to be type '" + expected_type_ptr->toString() + "'");
  }
}

Type* VarExprAST::generate(string& code) const
{
  Var* var_ptr = getVar();
  code += var_ptr->getCode();
  return var_ptr->getType();
}
