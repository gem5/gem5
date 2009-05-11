
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
 * TransistionDeclAST.h
 *
 * Description:
 *
 * $Id: TransitionDeclAST.h,v 3.2 2003/07/10 18:08:07 milo Exp $
 *
 */

#ifndef TransitionDeclAST_H
#define TransitionDeclAST_H

#include "mem/slicc/slicc_global.hh"
#include "mem/slicc/ast/DeclAST.hh"
#include "mem/slicc/ast/StatementListAST.hh"

class TransitionDeclAST : public DeclAST {
public:
  // Constructors
  TransitionDeclAST(Vector<string>* state_list_ptr,
                    Vector<string>* event_list_ptr,
                    string* next_state_ptr,
                    PairListAST* pairs_ptr,
                    Vector<string>* action_list_ptr);

  // Destructor
  ~TransitionDeclAST();

  // Public Methods
  void generate();
  void print(ostream& out) const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  TransitionDeclAST(const TransitionDeclAST& obj);
  TransitionDeclAST& operator=(const TransitionDeclAST& obj);

  // Data Members (m_ prefix)
  Vector<string>* m_state_list_ptr;
  Vector<string>* m_event_list_ptr;
  string* m_next_state_ptr;
  Vector<string>* m_action_list_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const TransitionDeclAST& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const TransitionDeclAST& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TransitionDeclAST_H
