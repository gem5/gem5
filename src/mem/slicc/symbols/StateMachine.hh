
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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "mem/slicc/slicc_global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/gems_common/Map.hh"
#include "mem/slicc/symbols/Symbol.hh"

class Transition;
class Event;
class State;
class Action;
class Var;
class Func;

class StateMachine : public Symbol {
public:
  // Constructors
  StateMachine(string ident, const Location& location, const Map<string, string>& pairs);

  // Destructor
  ~StateMachine();

  // Public Methods

  // Add items to the state machine
  //  void setMachine(string ident, const Map<string, string>& pairs);
  void addState(State* state_ptr);
  void addEvent(Event* event_ptr);
  void addAction(Action* action_ptr);
  void addTransition(Transition* trans_ptr);
  void addInPort(Var* var) { m_in_ports.insertAtBottom(var); }
  void addFunc(Func* func);

  // Accessors to vectors
  const State& getState(int index) const { return *m_states[index]; }
  const Event& getEvent(int index) const { return *m_events[index]; }
  const Action& getAction(int index) const { return *m_actions[index]; }
  const Transition& getTransition(int index) const { return *m_transitions[index]; }
  const Transition* getTransPtr(int stateIndex, int eventIndex) const;

  // Accessors for size of vectors
  int numStates() const { return m_states.size(); }
  int numEvents() const { return m_events.size(); }
  int numActions() const { return m_actions.size(); }
  int numTransitions() const { return m_transitions.size(); }

  void buildTable();  // Needs to be called before accessing the table

  // Code generator methods
  void writeCFiles(string path) const;
  void writeHTMLFiles(string path) const;

  void print(ostream& out) const { out << "[StateMachine: " << toString() << "]" << endl; }
private:
  // Private Methods
  void checkForDuplicate(const Symbol& sym) const;

  int getStateIndex(State* state_ptr) const { return m_state_map.lookup(state_ptr); }
  int getEventIndex(Event* event_ptr) const { return m_event_map.lookup(event_ptr); }

  // Private copy constructor and assignment operator
  //  StateMachine(const StateMachine& obj);
  //  StateMachine& operator=(const StateMachine& obj);

  void printControllerH(ostream& out, string component) const;
  void printControllerC(ostream& out, string component) const;
  void printCWakeup(ostream& out, string component) const;
  void printCSwitch(ostream& out, string component) const;
  void printProfilerH(ostream& out, string component) const;
  void printProfilerC(ostream& out, string component) const;

  void printHTMLTransitions(ostream& out, int active_state) const;

  // Data Members (m_ prefix)
  Vector<State*> m_states;
  Vector<Event*> m_events;
  Vector<Action*> m_actions;
  Vector<Transition*> m_transitions;
  Vector<Func*> m_internal_func_vec;

  Map<State*, int> m_state_map;
  Map<Event*, int> m_event_map;

  Vector<Var*> m_in_ports;

  // Table variables
  bool m_table_built;
  Vector<Vector<Transition*> > m_table;

};

// Output operator declaration
ostream& operator<<(ostream& out, const StateMachine& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const StateMachine& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //STATEMACHINE_H
