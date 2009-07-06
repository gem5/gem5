
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
 * Transition.hh
 *
 * Description:
 *
 * $Id$
 *
 * */

#ifndef TRANSITION_H
#define TRANSITION_H

#include "mem/slicc/slicc_global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/slicc/symbols/Symbol.hh"

class State;
class Event;
class Action;
class Var;

class Transition : public Symbol {
public:
  // Constructors
  Transition(string state, string event, string nextState,
             const Vector<string>& actionList,
             const Location& location,
             const Map<string, string>& pairMap);
  // Destructor
  ~Transition() { }

  // Public Methods
  State* getStatePtr() const { assert(m_statePtr != NULL); return m_statePtr; }
  Event* getEventPtr() const { assert(m_eventPtr != NULL); return m_eventPtr; }
  State* getNextStatePtr() const { assert(m_nextStatePtr != NULL); return m_nextStatePtr; }

  //  int getStateIndex() const { assert(m_statePtr != NULL); return m_statePtr->getIndex(); }
  //  int getEventIndex() const { assert(m_eventPtr != NULL); return m_eventPtr->getIndex(); }
  //  int getNextStateIndex() const { assert(m_nextStatePtr != NULL); return m_nextStatePtr->getIndex(); }
  void checkIdents(const Vector<State*>& states,
                   const Vector<Event*>& events,
                   const Vector<Action*>& actions);

  const string& getStateShorthand() const;
  const string& getEventShorthand() const;
  const string& getNextStateShorthand() const;
  string getActionShorthands() const;
  const Vector<Action*>& getActions() const { assert(m_actionPtrsValid); return m_actionPtrs; }
  const Map<Var*, string>& getResources() const { assert(m_actionPtrsValid); return m_resources; }

  void print(ostream& out) const;

  // Default copy constructor and assignment operator
  // Transition(const Transition& obj);
  // Transition& operator=(const Transition& obj);
private:
  // Private Methods
  Event* findIndex(const Vector<Event*>& vec, string ident);
  State* findIndex(const Vector<State*>& vec, string ident);
  Action* findIndex(const Vector<Action*>& vec, string ident);

  // Data Members (m_ prefix)
  string m_state;
  string m_event;
  string m_nextState;

  State* m_statePtr;
  Event* m_eventPtr;
  State* m_nextStatePtr;

  Vector<string> m_actionList;
  Vector<Action*> m_actionPtrs;
  Map<Var*, string> m_resources;
  bool m_actionPtrsValid;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Transition& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Transition& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TRANSITION_H
