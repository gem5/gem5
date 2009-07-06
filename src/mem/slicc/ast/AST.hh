
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
 * AST.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef AST_H
#define AST_H

#include "mem/slicc/slicc_global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/gems_common/Map.hh"
#include "mem/slicc/ast/Location.hh"
#include "mem/slicc/symbols/SymbolTable.hh"

class AST {
public:
  // Constructors
  AST(Map<string, string> pairs) { m_pairs = pairs; };
  AST() {};

  // Destructor
  virtual ~AST() {};

  // Public Methods
  virtual void print(ostream& out) const = 0;
  void error(string err_msg) const { m_location.error(err_msg); };
  string embedError(string err_msg) const { return m_location.embedError(err_msg); };
  void warning(string err_msg) const { m_location.warning(err_msg); };

  const Location& getLocation() const { return m_location; };

  const Map<string, string>& getPairs() const { return m_pairs; };
  Map<string, string>& getPairs() { return m_pairs; };

private:
  // Private Methods

  // Private copy constructor and assignment operator
  //  AST(const AST& obj);
  //  AST& operator=(const AST& obj);

  // Data Members (m_ prefix)
  Location m_location;
  Map<string, string> m_pairs;
};

// Output operator declaration
ostream& operator<<(ostream& out, const AST& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const AST& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //AST_H
