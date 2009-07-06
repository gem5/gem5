
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
 * FormalParamAST.C
 *
 * Description: See FormalParamAST.hh
 *
 * $Id: FormalParamAST.C,v 3.1 2000/10/05 21:22:20 milo Exp $
 *
 */

#include "mem/slicc/ast/FormalParamAST.hh"
#include "mem/slicc/ast/StatementAST.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

FormalParamAST::~FormalParamAST()
{
  delete m_ident_ptr;
  delete m_type_ast_ptr;
}

Type* FormalParamAST::generate(string& code) const
{
  string param = "param_" + *m_ident_ptr;

  Type* type_ptr = m_type_ast_ptr->lookupType();
  code += type_ptr->cIdent();
  code += " ";
  code += param;

  // Add to symbol table
  g_sym_table.newSym(new Var(*m_ident_ptr, getLocation(), type_ptr, param, getPairs()));
  return type_ptr;
}
