
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
 * ExprAST.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef EXPRAST_H
#define EXPRAST_H

#include "mem/slicc/slicc_global.hh"
#include "mem/slicc/ast/AST.hh"


class ExprAST : public AST {
public:
  // Constructors
  ExprAST() : AST() { }

  // Destructor
  virtual ~ExprAST() { }

  // Public Methods
  virtual Type* generate(string& code) const = 0;
  virtual void findResources(Map<Var*, string>& resource_list) const {} // The default is no resources

  // void print(ostream& out) const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  // ExprAST(const ExprAST& obj);
  // ExprAST& operator=(const ExprAST& obj);

  // Data Members (m_ prefix)

};

// Output operator declaration
ostream& operator<<(ostream& out, const ExprAST& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const ExprAST& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //EXPRAST_H
