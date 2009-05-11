
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
 */

#ifndef SYMBOL_H
#define SYMBOL_H

#include "slicc_global.hh"
#include "Map.hh"
#include "Location.hh"

class Symbol {
public:
  // Constructors
  Symbol(string id, const Location& location, const Map<string, string>& pairs);
  Symbol(string id, const Location& location);
  // Destructor
  virtual ~Symbol() { }

  // Public Methods
  void error(string err_msg) const { m_location.error(err_msg); }
  void warning(string err_msg) const { m_location.warning(err_msg); }
  const Location& getLocation() const { return m_location; }

  const string& toString() const { return m_id; }

  const string& getIdent() const { return m_id; }
  const string& getShorthand() const { return lookupPair("short"); }
  const string& getDescription() const { return lookupPair("desc"); }

  void markUsed() { m_used = true; }
  bool wasUsed() { return m_used; }

  bool existPair(const string& key) const { return m_pairs.exist(key); }
  const string& lookupPair(const string& key) const;
  void addPair(const string& key, const string& value);

  //  virtual string getCode() const = 0;
  virtual void writeCFiles(string path) const {}
  virtual void writeHTMLFiles(string path) const {}
  virtual void print(ostream& out) const { out << "[Symbol: " << getIdent() << "]"; }

private:
  // Private Methods

  // Private copy constructor and assignment operator
  // Symbol(const Symbol& obj);
  // Symbol& operator=(const Symbol& obj);

  // Data Members (m_ prefix)
  string m_id;
  Map<string, string> m_pairs;
  Location m_location;
  bool m_used;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Symbol& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Symbol& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //SYMBOL_H
