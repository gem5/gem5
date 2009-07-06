
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
 * Func.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef FUNC_H
#define FUNC_H

#include "mem/slicc/slicc_global.hh"
#include "mem/slicc/symbols/Type.hh"
class StateMachine;

class Func : public Symbol {
public:
  // Constructors
  Func(string id, const Location& location,
       Type* type_ptr, const Vector<Type*>& param_type_vec, const Vector<string>& param_string_vec,
       string body, const Map<string, string>& pairs, StateMachine* machine_ptr);

  // Destructor
  ~Func() {}

  // Public Methods
  string cIdent() const { return m_c_ident; }
  const Vector<Type*>& getParamTypes() const { return m_param_type_vec; }
  Type* getReturnType() const { return m_type_ptr; }
  void writeCFiles(string path) const;
  void funcPrototype(string& code) const;
  bool isExternal() const { return existPair("external"); }
  bool isInternalMachineFunc() const { return m_isInternalMachineFunc; }
  void print(ostream& out) const;
private:
  // Private Methods

  // Private copy constructor and assignment operator
  Func(const Func& obj);
  Func& operator=(const Func& obj);

  // Data Members (m_ prefix)
  Type* m_type_ptr;
  Vector<Type*> m_param_type_vec;
  Vector<string> m_param_string_vec;
  string m_body;
  string m_c_ident;
  string m_machineStr;
  bool m_isInternalMachineFunc;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Func& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Func& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //FUNC_H
