
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
 * StatementListAST.C
 *
 * Description: See StatementListAST.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/StatementListAST.hh"

StatementListAST::StatementListAST(Vector<StatementAST*>* vec_ptr)
  : AST()
{
  assert(vec_ptr != NULL);
  m_vec_ptr = vec_ptr;
}

// Singleton constructor.
StatementListAST::StatementListAST(StatementAST* statement_ptr)
  : AST()
{
  assert(statement_ptr != NULL);
  m_vec_ptr = new Vector<StatementAST*>;
  m_vec_ptr->insertAtTop(statement_ptr);
}

StatementListAST::~StatementListAST()
{
  int size = m_vec_ptr->size();
  for(int i=0; i<size; i++) {
    delete (*m_vec_ptr)[i];
  }
  delete m_vec_ptr;
}

void StatementListAST::generate(string& code, Type* return_type_ptr) const
{
  int size = m_vec_ptr->size();
  for(int i=0; i<size; i++) {
    (*m_vec_ptr)[i]->generate(code, return_type_ptr);
  }
}

void StatementListAST::findResources(Map<Var*, string>& resource_list) const
{
  int size = m_vec_ptr->size();
  for(int i=0; i<size; i++) {
    (*m_vec_ptr)[i]->findResources(resource_list);
  }
}

void StatementListAST::print(ostream& out) const
{
  assert(m_vec_ptr != NULL);
  out << "[StatementListAST: " << *m_vec_ptr << "]";
}
