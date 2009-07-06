
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
 * SymbolTable.cc
 *
 * Description: See SymbolTable.hh
 *
 * $Id$
 *
 * */

#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/generator/fileio.hh"
#include "mem/slicc/generator/html_gen.hh"
#include "mem/slicc/generator/mif_gen.hh"
#include "mem/slicc/symbols/Action.hh"

SymbolTable g_sym_table;

SymbolTable::SymbolTable()
{
  m_sym_map_vec.setSize(1);
  m_depth = 0;

  {
    Map<string, string> pairs;
    pairs.add("enumeration", "yes");
    newSym(new Type("MachineType", Location(), pairs));
  }

  {
    Map<string, string> pairs;
    pairs.add("primitive", "yes");
    pairs.add("external", "yes");
    newSym(new Type("void", Location(), pairs));
  }
}

SymbolTable::~SymbolTable()
{
  int size = m_sym_vec.size();
  for(int i=0; i<size; i++) {
    delete m_sym_vec[i];
  }
}

void SymbolTable::newSym(Symbol* sym_ptr)
{
  registerSym(sym_ptr->toString(), sym_ptr);
  m_sym_vec.insertAtBottom(sym_ptr);  // Holder for the allocated Sym objects.
}

void SymbolTable::newMachComponentSym(Symbol* sym_ptr)
{
  // used to cheat-- that is, access components in other machines
  StateMachine* mach_ptr = getStateMachine("current_machine");
  if (mach_ptr != NULL) {
    m_machine_component_map_vec.lookup(mach_ptr->toString()).add(sym_ptr->toString(), sym_ptr);
  }
}

Var* SymbolTable::getMachComponentVar(string mach, string ident)
{
  Symbol* s = m_machine_component_map_vec.lookup(mach).lookup(ident);
  return dynamic_cast<Var*>(s);
}


void SymbolTable::registerSym(string id, Symbol* sym_ptr)
{

  // Check for redeclaration (in the current frame only)
  if (m_sym_map_vec[m_depth].exist(id)) {
    sym_ptr->error("Symbol '" + id + "' redeclared in same scope.");
  }
  // FIXME - warn on masking of a declaration in a previous frame
  m_sym_map_vec[m_depth].add(id, sym_ptr);
}

void SymbolTable::registerGlobalSym(string id, Symbol* sym_ptr)
{
  // Check for redeclaration (global frame only)
  if (m_sym_map_vec[0].exist(id)) {
    sym_ptr->error("Global symbol '" + id + "' redeclared in global scope.");
  }
  m_sym_map_vec[0].add(id, sym_ptr);
}

Symbol* SymbolTable::getSym(string ident) const
{
  for (int i=m_depth; i>=0; i--) {
    if (m_sym_map_vec[i].exist(ident)) {
      return m_sym_map_vec[i].lookup(ident);
    }
  }
  return NULL;
}

void SymbolTable::newCurrentMachine(StateMachine* sym_ptr)
{
  registerGlobalSym(sym_ptr->toString(), sym_ptr);
  registerSym("current_machine", sym_ptr);
  m_sym_vec.insertAtBottom(sym_ptr);  // Holder for the allocated Sym objects.

  Map<string, Symbol*> m;
  m_machine_component_map_vec.add(sym_ptr->toString(),m);

}

Type* SymbolTable::getType(string ident) const
{
  return dynamic_cast<Type*>(getSym(ident));
}

Var* SymbolTable::getVar(string ident) const
{
  return dynamic_cast<Var*>(getSym(ident));
}

Func* SymbolTable::getFunc(string ident) const
{
  return dynamic_cast<Func*>(getSym(ident));
}

StateMachine* SymbolTable::getStateMachine(string ident) const
{
  return dynamic_cast<StateMachine*>(getSym(ident));
}

void SymbolTable::pushFrame()
{
  m_depth++;
  m_sym_map_vec.expand(1);
  m_sym_map_vec[m_depth].clear();
}

void SymbolTable::popFrame()
{
  m_depth--;
  assert(m_depth >= 0);
  m_sym_map_vec.expand(-1);
}

void SymbolTable::writeCFiles(string path) const
{
  int size = m_sym_vec.size();
  {
    // Write the Types.hh include file for the types
    ostringstream sstr;
    sstr << "/** Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< " */" << endl;
    sstr << endl;
    sstr << "#include \"mem/ruby/slicc_interface/RubySlicc_includes.hh\"" << endl;
    for(int i=0; i<size; i++) {
      Type* type = dynamic_cast<Type*>(m_sym_vec[i]);
      if (type != NULL && !type->isPrimitive()) {
        sstr << "#include \"mem/protocol/" << type->cIdent() << ".hh" << "\"" << endl;
      }
    }
    conditionally_write_file(path + "/Types.hh", sstr);
  }

  // Write all the symbols
  for(int i=0; i<size; i++) {
    m_sym_vec[i]->writeCFiles(path + '/');
  }

  writeControllerFactory(path);
}

void SymbolTable::writeControllerFactory(string path) const
{
  ostringstream sstr;
  int size = m_sym_vec.size();

  sstr << "/** \\file ControllerFactory.hh " << endl;
  sstr << "  * Auto generatred C++ code started by " << __FILE__ << ":" << __LINE__ << endl;
  sstr << "  */" << endl << endl;

  sstr << "#ifndef CONTROLLERFACTORY_H" << endl;
  sstr << "#define CONTROLLERFACTORY_H" << endl;
  sstr << endl;

  Vector< string > controller_types;

  // includes
  sstr << "#include <string>" << endl;
  sstr << "class Network;" << endl;
  sstr << "class AbstractController;" << endl;
  sstr << endl;

  sstr << "class ControllerFactory {" << endl;
  sstr << "public:" << endl;
  sstr << "  static AbstractController* createController(const std::string & controller_type, const std::string & name);" << endl;
  sstr << "};" << endl;
  sstr << endl;

  sstr << "#endif // CONTROLLERFACTORY_H" << endl;
  conditionally_write_file(path + "/ControllerFactory.hh", sstr);

  // ControllerFactory.cc file

  sstr.str("");

  sstr << "/** \\file ControllerFactory.cc " << endl;
  sstr << "  * Auto generatred C++ code started by " << __FILE__ << ":" << __LINE__ << endl;
  sstr << "  */" << endl << endl;

  // includes
  sstr << "#include \"mem/protocol/ControllerFactory.hh\"" << endl;
  sstr << "#include \"mem/ruby/slicc_interface/AbstractController.hh\"" << endl;
  sstr << "#include \"mem/protocol/MachineType.hh\"" << endl;
  for(int i=0; i<size; i++) {
    StateMachine* machine = dynamic_cast<StateMachine*>(m_sym_vec[i]);
    if (machine != NULL) {
      sstr << "#include \"mem/protocol/" << machine->getIdent() << "_Controller.hh\"" << endl;
      controller_types.insertAtBottom(machine->getIdent());
    }
  }
  sstr << endl;

  sstr << "AbstractController* ControllerFactory::createController(const std::string & controller_type, const std::string & name) {" << endl;
  for (int i=0;i<controller_types.size();i++) {
    sstr << "    if (controller_type == \"" << controller_types[i] << "\")" << endl;
    sstr << "      return new " << controller_types[i] << "_Controller(name);" << endl;
  }
  sstr << "  assert(0); // invalid controller type" << endl;
  sstr << "  return NULL;" << endl;
  sstr << "}" << endl;
  conditionally_write_file(path + "/ControllerFactory.cc", sstr);
}

Vector<StateMachine*> SymbolTable::getStateMachines() const
{
  Vector<StateMachine*> machine_vec;
  int size = m_sym_vec.size();
  for(int i=0; i<size; i++) {
    StateMachine* type = dynamic_cast<StateMachine*>(m_sym_vec[i]);
    if (type != NULL) {
      machine_vec.insertAtBottom(type);
    }
  }
  return machine_vec;
}

void SymbolTable::writeHTMLFiles(string path) const
{
  // Create index.html
  {
    ostringstream out;
    createHTMLindex(path, out);
    conditionally_write_file(path + "index.html", out);
  }

  // Create empty.html
  {
    ostringstream out;
    out << "<HTML></HTML>";
    conditionally_write_file(path + "empty.html", out);
  }

  // Write all the symbols
  int size = m_sym_vec.size();
  for(int i=0; i<size; i++) {
    m_sym_vec[i]->writeHTMLFiles(path);
  }
}

void write_file(string filename, ostringstream& sstr)
{
  ofstream out;

  out.open(filename.c_str());
  out << sstr.str();
  out.close();
}

void SymbolTable::writeMIFFiles(string path) const
{
  int size = m_sym_vec.size();
  for(int i=0; i<size; i++) {
    ostringstream states, events, actions, transitions;
    StateMachine* machine = dynamic_cast<StateMachine*>(m_sym_vec[i]);
    if (machine != NULL) {
      printStateTableMIF(*machine, states);
      write_file(path + machine->getIdent() + "_states.mif", states);
      printEventTableMIF(*machine, events);
      write_file(path + machine->getIdent() + "_events.mif", events);
      printActionTableMIF(*machine, actions);
      write_file(path + machine->getIdent() + "_actions.mif", actions);
      printTransitionTableMIF(*machine, transitions);
      write_file(path + machine->getIdent() + "_transitions.mif", transitions);
    }
  }
}


void SymbolTable::print(ostream& out) const
{
  out << "[SymbolTable]";  // FIXME
}
