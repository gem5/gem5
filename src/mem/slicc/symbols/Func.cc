
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
 * Func.cc
 *
 * Description: See Func.hh
 *
 * $Id$
 *
 */

#include "mem/slicc/symbols/Func.hh"
#include "mem/slicc/symbols/SymbolTable.hh"
#include "mem/slicc/generator/fileio.hh"
#include "mem/slicc/symbols/StateMachine.hh"

Func::Func(string id, const Location& location,
           Type* type_ptr, const Vector<Type*>& param_type_vec,
           const Vector<string>& param_string_vec, string body,
           const Map<string, string>& pairs, StateMachine* machine_ptr)
  : Symbol(id, location, pairs)
{
  m_type_ptr = type_ptr;
  m_param_type_vec = param_type_vec;
  m_param_string_vec = param_string_vec;
  m_body = body;
  m_isInternalMachineFunc = false;

  if (machine_ptr == NULL) {
    m_c_ident = id;
  } else if (existPair("external") || existPair("primitive")) {
    m_c_ident = id;
  } else {
    m_machineStr = machine_ptr->toString();
    m_c_ident = m_machineStr + "_" + id;  // Append with machine name
    m_isInternalMachineFunc = true;
  }
}

void Func::funcPrototype(string& code) const
{
  if (isExternal()) {
    // Do nothing
  } else {
    string return_type = m_type_ptr->cIdent();
    Type* void_type_ptr = g_sym_table.getType("void");
    if (existPair("return_by_ref") && (m_type_ptr != void_type_ptr)) {
      return_type += "&";
    }
    code += return_type + " " + cIdent() + "(";
    int size = m_param_string_vec.size();
    for(int i=0; i<size; i++) {
      // Generate code
      if (i != 0) {
        code += ", ";
      }
      code += m_param_string_vec[i];
    }
    code += ");\n";
  }
}

// This write a function of object Chip
void Func::writeCFiles(string path) const
{
  if (isExternal()) {
    // Do nothing
  } else {
    ostringstream out;

    // Header
    out << "/** Auto generated C++ code started by "<<__FILE__<<":"<<__LINE__<< " */" << endl;
    out << endl;
    out << "#include \"mem/protocol/Types.hh\"" << endl;
    out << "#include \"mem/protocol/Chip.hh\"" << endl;
    if (m_isInternalMachineFunc) {
      out << "#include \"" << m_machineStr << "_Controller.hh\"" << endl;
    }
    out << endl;

    // Generate function header
    string code;
    Type* void_type_ptr = g_sym_table.getType("void");
    string return_type = m_type_ptr->cIdent();
    code += return_type;
    if (existPair("return_by_ref") && m_type_ptr != void_type_ptr) {
      code += "&";
    }
    if (!m_isInternalMachineFunc) {
      code += " Chip::" + cIdent() + "(";
    } else {
      code += " " + m_machineStr + "_Controller::" + cIdent() + "(";
    }
    int size = m_param_type_vec.size();
    for(int i=0; i<size; i++) {
      // Generate code
      if (i != 0) {
        code += ", ";
      }
      code += m_param_string_vec[i];
    }
    code += ")";

    // Function body
    code += "\n{\n";
    code += m_body;
    code += "}\n";
    out << code << endl;

    // Write it out
    conditionally_write_file(path + cIdent() + ".cc", out);
  }
}

void Func::print(ostream& out) const
{
}
