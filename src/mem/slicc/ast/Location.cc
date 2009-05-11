
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
 * Location.C
 *
 * Description: See Location.h
 *
 * $Id: Location.C,v 3.3 2004/05/30 22:19:02 kmoore Exp $
 *
 */

#include "mem/slicc/ast/Location.hh"

int g_line_number = 0;
string g_file_name("");

Location::Location()
{
  m_file_name = g_file_name;
  m_line_number = g_line_number;

  ostringstream sstr;
  sstr << getLineNumber();
  m_line_number_str = sstr.str();
}

void Location::error(string err_msg) const
{
  cerr << endl;
  cerr << toString() << ": Error: " << err_msg << endl;
  exit(1);
}

string Location::embedError(string err_msg) const
{
  string code;
  code += "cerr << \"Runtime Error at ";
  code += toString() + ", Ruby Time: \" << ";
  code += "g_eventQueue_ptr->getTime() << \": \" << ";
  code += err_msg;
  code += " << \", PID: \" << getpid() << endl;\n";
  code += "char c; cerr << \"press return to continue.\" << endl; cin.get(c); abort();\n";

  return code;
}

void Location::warning(string err_msg) const
{
  cerr << toString() << ": Warning: "
       << err_msg << endl;
}

string Location::toString() const
{
  return m_file_name + ":" + m_line_number_str;
}
