/*
 * Copyright (c) 1999-2005 Mark D. Hill and David A. Wood
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

#include <cassert>

#include "mem/gems_common/util.hh"

// Split a string into a head and tail strings on the specified
// character.  Return the head and the string passed in is modified by
// removing the head, leaving just the tail.

string string_split(string& str, char split_character)
{
  string head = "";
  string tail = "";

  unsigned counter = 0;
  while(counter < str.size()) {
    if (str[counter] == split_character) {
      counter++;
      break;
    } else {
      head += str[counter];
    }
    counter++;
  }

  while(counter < str.size()) {
    tail += str[counter];
    counter++;
  }
  str = tail;
  return head;
}

string bool_to_string(bool value)
{
  if (value) {
    return "true";
  } else {
    return "false";
  }
}

string int_to_string(int n, bool zero_fill, int width)
{
  ostringstream sstr;
  if(zero_fill) {
    sstr << setw(width) << setfill('0') << n;
  } else {
    sstr << n;
  }
  string str = sstr.str();
  return str;
}

float string_to_float(string& str)
{
  stringstream sstr(str);
  float ret;
  sstr >> ret;
  return ret;
}

// Log functions
int log_int(long long n)
{
  assert(n > 0);
  int counter = 0;
  while (n >= 2) {
    counter++;
    n = n>>(long long)(1);
  }
  return counter;
}

bool is_power_of_2(long long n)
{
  return (n == ((long long)(1) << log_int(n)));
}

