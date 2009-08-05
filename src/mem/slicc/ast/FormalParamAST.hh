
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
 * FormalParamAST.hh
 *
 * Description:
 *
 * $Id: FormalParamAST.hh,v 3.1 2001/12/12 01:00:15 milo Exp $
 *
 */

#ifndef FORMALPARAMAST_H
#define FORMALPARAMAST_H

#include "mem/slicc/slicc_global.hh"
#include "mem/slicc/ast/AST.hh"

class TypeAST;


class FormalParamAST : public AST {
public:
  // Constructors
  FormalParamAST(TypeAST* type_ast_ptr, string* ident_ptr) : AST() { m_type_ast_ptr = type_ast_ptr; m_ident_ptr = ident_ptr; }

  // Destructor
  ~FormalParamAST();

  // Public Methods
  Type* generate(string& code) const;
  void print(ostream& out) const { out << "[FormalParamAST: " << *m_ident_ptr << "]"; }
  string getName() const { return *m_ident_ptr; }
  string getTypeName() const;
  Type* getType() const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  FormalParamAST(const FormalParamAST& obj);
  FormalParamAST& operator=(const FormalParamAST& obj);

  // Data Members (m_ prefix)
  string* m_ident_ptr;
  TypeAST* m_type_ast_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const FormalParamAST& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const FormalParamAST& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //FORMALPARAMAST_H
