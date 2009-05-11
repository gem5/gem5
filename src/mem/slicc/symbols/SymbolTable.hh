
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
 * SymbolTable.h
 *
 * Description:
 *
 * $Id$
 *
 * */

#ifndef SYMBOLTABLE_H
#define SYMBOLTABLE_H

#include "slicc_global.hh"
#include "Map.hh"
#include "Vector.hh"

#include "Symbol.hh"
#include "Type.hh"
#include "Var.hh"
#include "Func.hh"
#include "StateMachine.hh"

class SymbolTable;

extern SymbolTable g_sym_table;

class SymbolTable {
public:
  // Constructors
  SymbolTable();

  // Destructor
  ~SymbolTable();

  // Public Methods
  void newSym(Symbol* sym_ptr);
  void registerSym(string id, Symbol* sym_ptr);
  Symbol* getSym(string id) const;

  // used to cheat-- that is, access components in other machines
  void newMachComponentSym(Symbol* sym_ptr);
  Var* getMachComponentVar(string mach, string ident);

  void newCurrentMachine(StateMachine* machine_ptr);
  StateMachine* getStateMachine(string ident) const;
  StateMachine* getStateMachine() const { return getStateMachine("current_machine"); }
  Type* getType(string ident) const;

  Var* getVar(string ident) const;
  Func* getFunc(string ident) const;

  void pushFrame();
  void popFrame();

  Vector<StateMachine*> getStateMachines() const;

  void writeCFiles(string path) const;
  void writeHTMLFiles(string path) const;
  void writeMIFFiles(string path) const;

  void print(ostream& out) const;
private:
  // Private Methods
  void registerGlobalSym(string id, Symbol* sym_ptr);
  void writeChipFiles(string path) const;

  // Private copy constructor and assignment operator
  SymbolTable(const SymbolTable& obj);
  SymbolTable& operator=(const SymbolTable& obj);

  // Data Members (m_ prefix)
  Vector<Symbol*> m_sym_vec;
  Vector<Map<string, Symbol*> > m_sym_map_vec;
  Map<string, Map<string, Symbol*> > m_machine_component_map_vec;
  int m_depth;
};

// Output operator declaration
ostream& operator<<(ostream& out, const SymbolTable& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const SymbolTable& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SYMBOLTABLE_H
