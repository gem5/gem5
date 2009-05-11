
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
 * */

#include "Transition.hh"
#include "State.hh"
#include "Event.hh"
#include "Action.hh"
#include "util.hh"
#include "Var.hh"

Transition::Transition(string state, string event, string nextState,
                       const Vector<string>& actionList,
                       const Location& location,
                       const Map<string, string>& pairMap)
  : Symbol(state + "|" + event, location, pairMap)
{
  m_state = state;
  m_event = event;
  m_nextState = nextState;
  m_actionList = actionList;

  // Ptrs are undefined at this point
  m_statePtr = NULL;
  m_eventPtr = NULL;
  m_nextStatePtr = NULL;
  m_actionPtrsValid = false;
}

void Transition::checkIdents(const Vector<State*>& states,
                             const Vector<Event*>& events,
                             const Vector<Action*>& actions)
{
  m_statePtr = findIndex(states, m_state);
  m_eventPtr = findIndex(events, m_event);
  m_nextStatePtr = findIndex(states, m_nextState);

  for(int i=0; i < m_actionList.size(); i++) {
    Action* action_ptr = findIndex(actions, m_actionList[i]);
    int size = action_ptr->getResources().keys().size();
    for (int j=0; j < size; j++) {
      Var* var_ptr = action_ptr->getResources().keys()[j];
      if (var_ptr->getType()->cIdent() != "DNUCAStopTable") {
        int num = atoi((action_ptr->getResources().lookup(var_ptr)).c_str());
        if (m_resources.exist(var_ptr)) {
          num += atoi((m_resources.lookup(var_ptr)).c_str());
        }
        m_resources.add(var_ptr, int_to_string(num));
      } else {
        m_resources.add(var_ptr, action_ptr->getResources().lookup(var_ptr));
      }
    }
    m_actionPtrs.insertAtBottom(action_ptr);
  }
  m_actionPtrsValid = true;
}

const string& Transition::getStateShorthand() const
{
  assert(m_statePtr != NULL);
  return m_statePtr->getShorthand();
}

const string& Transition::getEventShorthand() const
{
  assert(m_eventPtr != NULL);
  return m_eventPtr->getShorthand();
}

const string& Transition::getNextStateShorthand() const
{
  assert(m_nextStatePtr != NULL);
  return m_nextStatePtr->getShorthand();
}

string Transition::getActionShorthands() const
{
  assert(m_actionPtrsValid);
  string str;
  int numActions = m_actionPtrs.size();
  for (int i=0; i<numActions; i++) {
    str += m_actionPtrs[i]->getShorthand();
  }
  return str;
}

void Transition::print(ostream& out) const
{
  out << "[Transition: ";
  out << "(" << m_state;
  if (m_statePtr != NULL) {
    out << ":" << *m_statePtr;
  }
  out << ", " << m_event;
  if (m_eventPtr != NULL) {
    out << ":" << *m_eventPtr;
  }
  out << ") -> ";
  out << m_nextState;
  if (m_nextStatePtr != NULL) {
    out << ":" << *m_nextStatePtr;
  }
  out << ", ";
  out << m_actionList;
  out << "]";
}

Event* Transition::findIndex(const Vector<Event*>& vec, string ident)
{
  int size = vec.size();
  for(int i=0; i<size; i++) {
    if (ident == vec[i]->getIdent()) {
      return vec[i];
    }
  }
  error("Event not found: " + ident);
  return NULL;
}

State* Transition::findIndex(const Vector<State*>& vec, string ident)
{
  int size = vec.size();
  for(int i=0; i<size; i++) {
    if (ident == vec[i]->getIdent()) {
      return vec[i];
    }
  }
  error("State not found: " + ident);
  return NULL;
}

Action* Transition::findIndex(const Vector<Action*>& vec, string ident)
{
  int size = vec.size();
  for(int i=0; i<size; i++) {
    if (ident == vec[i]->getIdent()) {
      return vec[i];
    }
  }
  error("Action not found: " + ident);
  return NULL;
}

