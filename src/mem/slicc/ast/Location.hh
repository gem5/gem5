
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
 * Location.h
 *
 * Description:
 *
 * $Id: Location.h,v 3.1 2001/12/12 01:00:20 milo Exp $
 *
 */

#ifndef LOCATION_H
#define LOCATION_H

#include "mem/slicc/slicc_global.hh"

extern int g_line_number;
extern string g_file_name;

class Location {
public:
  // Constructors
  Location();

  // Destructor
  //~Location();

  // Public Methods

  void print(ostream& out) const;
  void error(string err_msg) const;
  string embedError(string err_msg) const;
  void warning(string err_msg) const;
  string toString() const;

private:
  // Private Methods
  const string& getFileName() const { return m_file_name; }
  int getLineNumber() const { return m_line_number; }
  string getLineNumberStr() const { return m_line_number_str; }

  // Private copy constructor and assignment operator
  //Location(const Location& obj);
  //Location& operator=(const Location& obj);

  // Data Members (m_ prefix)
  string m_file_name;
  int m_line_number;
  string m_line_number_str;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Location& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Location& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //LOCATION_H
