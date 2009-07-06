
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
 * Type.hh
 *
 * Description:
 *
 * $Id$
 *
 * */

#ifndef TYPE_H
#define TYPE_H

#include "mem/slicc/slicc_global.hh"
#include "mem/gems_common/Map.hh"
#include "mem/slicc/symbols/Symbol.hh"

class StateMachine;

class Type : public Symbol {
public:
  // Constructors
  Type(string id, const Location& location,
       const Map<string, string>& pairs,
       StateMachine* machine_ptr = NULL);

  // Destructor
  ~Type() {}

  // Public Methods
  string cIdent() const { return m_c_id; }
  string desc() const { return m_desc; }

  bool isPrimitive() const { return existPair("primitive"); }
  bool isNetworkMessage() const { return existPair("networkmessage"); }
  bool isMessage() const { return existPair("message"); }
  bool isBuffer() const { return existPair("buffer"); }
  bool isInPort() const { return existPair("inport"); }
  bool isOutPort() const { return existPair("outport"); }
  bool isEnumeration() const { return existPair("enumeration"); }
  bool isExternal() const { return existPair("external"); }
  bool isGlobal() const { return existPair("global"); }

  // The data members of this type - only valid for messages and SLICC
  // declared structures
  // Return false on error
  bool dataMemberAdd(string id, Type* type_ptr, Map<string, string>& pairs,
                     string* init_code);
  bool dataMemberExist(string id) const { return m_data_member_map.exist(id); }
  Type* dataMemberType(string id) const { return m_data_member_map.lookup(id); }

  // The methods of this type - only valid for external types
  // Return false on error
  bool methodAdd(string name, Type* return_type_ptr, const Vector<Type*>& param_type_vec);
  bool methodExist(string id) const { return m_method_return_type_map.exist(id); }

  string methodId(string name, const Vector<Type*>& param_type_vec);
  Type* methodReturnType(string id) const { return m_method_return_type_map.lookup(id); }
  const Vector<Type*>& methodParamType(string id) const { return m_method_param_type_map.lookup(id); }

  // The enumeration idents of this type - only valid for enums
  // Return false on error
  bool enumAdd(string id, Map<string, string> pairs);
  bool enumExist(string id) const { return m_enum_map.exist(id); }

  // Write the C output files
  void writeCFiles(string path) const;

  bool hasDefault() const { return existPair("default"); }
  string getDefault() const { return lookupPair("default"); }

  void print(ostream& out) const {}
private:
  // Private Methods

  void printTypeH(string path) const;
  void printTypeC(string path) const;
  void printEnumC(string path) const;
  void printEnumH(string path) const;

  // Private copy constructor and assignment operator
  Type(const Type& obj);
  Type& operator=(const Type& obj);

  // Data Members (m_ prefix)
  string m_c_id;
  string m_desc;

  // Data Members
  Map<string, Type*> m_data_member_map;
  Vector<string> m_data_member_ident_vec;
  Vector<Type*> m_data_member_type_vec;
  Vector<Map<string, string> > m_data_member_pairs_vec;
  Vector<string*> m_data_member_init_code_vec;
  // Needs pairs here

  // Methods
  Map<string, Type*> m_method_return_type_map;
  Map<string, Vector<Type*> > m_method_param_type_map;
  // Needs pairs here

  // Enum
  Map<string, bool> m_enum_map;
  Vector<string> m_enum_vec;
  Vector< Map < string, string > > m_enum_pairs;

  // MachineType Hack
  bool m_isMachineType;

};

// Output operator declaration
ostream& operator<<(ostream& out, const Type& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Type& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //TYPE_H
