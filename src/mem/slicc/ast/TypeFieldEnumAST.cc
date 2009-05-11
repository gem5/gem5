
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
 * TypeFieldEnumAST.C
 *
 * Description: See TypeFieldEnumAST.h
 *
 * $Id$
 *
 */

#include "mem/slicc/ast/TypeFieldEnumAST.hh"
#include "mem/slicc/symbols/State.hh"
#include "mem/slicc/symbols/Event.hh"

TypeFieldEnumAST::TypeFieldEnumAST(string* field_id_ptr,
                                   PairListAST* pairs_ptr)
  : TypeFieldAST(pairs_ptr)
{
  m_field_id_ptr = field_id_ptr;
  m_pairs_ptr = pairs_ptr;
}

TypeFieldEnumAST::~TypeFieldEnumAST()
{
  delete m_field_id_ptr;
}

void TypeFieldEnumAST::generate(Type *type_ptr)
{
  // Add enumeration
  if (!type_ptr->enumAdd(*m_field_id_ptr, m_pairs_ptr->getPairs())) {
    error("Duplicate enumeration: " + type_ptr->toString() + ":" + *m_field_id_ptr);
  }

  // Fill machine info
  StateMachine* machine_ptr = g_sym_table.getStateMachine();
  if (type_ptr->toString() == "State") {
    if (machine_ptr == NULL) {
      error("State declaration not part of a machine.");
    }
    machine_ptr->addState(new State(*m_field_id_ptr, getLocation(), getPairs()));
  }
  if (type_ptr->toString() == "Event") {
    if (machine_ptr == NULL) {
      error("Event declaration not part of a machine.");
    }
    machine_ptr->addEvent(new Event(*m_field_id_ptr, getLocation(), getPairs()));
  }
}

void TypeFieldEnumAST::print(ostream& out) const
{
  out << "[TypeFieldEnum: " << *m_field_id_ptr << "]";
}
