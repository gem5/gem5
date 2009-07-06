
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
 * TransitionDeclAST.C
 *
 * Description: See TransitionDeclAST.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/TransitionDeclAST.hh"
#include "mem/slicc/symbols/Transition.hh"

TransitionDeclAST::TransitionDeclAST(Vector<string>* state_list_ptr,
                                     Vector<string>* event_list_ptr,
                                     string* next_state_ptr,
                                     PairListAST* pairs_ptr,
                                     Vector<string>* action_list_ptr)
  : DeclAST(pairs_ptr)
{
  m_state_list_ptr = state_list_ptr;
  m_event_list_ptr = event_list_ptr;
  m_next_state_ptr = next_state_ptr;
  m_action_list_ptr = action_list_ptr;
}

TransitionDeclAST::~TransitionDeclAST()
{
  delete m_state_list_ptr;
  delete m_event_list_ptr;
  delete m_next_state_ptr;
  delete m_action_list_ptr;
}

void TransitionDeclAST::generate()
{
  Vector<string>& states = *m_state_list_ptr;
  Vector<string>& events = *m_event_list_ptr;

  StateMachine* machine_ptr = g_sym_table.getStateMachine();
  if (machine_ptr == NULL) {
    error("Transition declaration not part of a machine.");
  } else if (m_next_state_ptr == NULL) {
    for (int i=0; i<states.size(); i++) {
      for (int j=0; j<events.size(); j++) {
        machine_ptr->addTransition(new Transition(states[i], events[j], states[i], *m_action_list_ptr, getLocation(), getPairs()));
      }
    }
  } else {
    for (int i=0; i<states.size(); i++) {
      for (int j=0; j<events.size(); j++) {
        machine_ptr->addTransition(new Transition(states[i], events[j], *m_next_state_ptr, *m_action_list_ptr, getLocation(), getPairs()));
      }
    }
  }
}

void TransitionDeclAST::print(ostream& out) const
{
  out << "[TransitionDecl: ]";
}
