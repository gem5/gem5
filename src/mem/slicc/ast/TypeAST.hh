
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
 * TypeAST.h
 *
 * Description:
 *
 * $Id: TypeAST.h,v 3.2 2003/03/22 15:15:17 xu Exp $
 *
 */

#ifndef TYPEAST_H
#define TYPEAST_H

#include "slicc_global.hh"
#include "AST.hh"

class TypeAST : public AST {
public:
  // Constructors
  TypeAST(string* ident_ptr);

  // Destructor
  ~TypeAST();

  // Public Methods
  string toString() const;
  Type* lookupType() const;

  virtual void print(ostream& out) const {}
private:
  // Private Methods

  // Private copy constructor and assignment operator
  TypeAST(const TypeAST& obj);
  TypeAST& operator=(const TypeAST& obj);

  // Data Members (m_ prefix)
  string* m_ident_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const TypeAST& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const TypeAST& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TYPEAST_H
