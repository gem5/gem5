
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

#ifndef SLICC_GLOBAL_H
#define SLICC_GLOBAL_H

#include <assert.h> /* slicc needs to include this in order to use classes in
                     * ../common directory.
                     */

#include "std-includes.hh"
#include "Map.hh"

typedef unsigned char uint8;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef signed char int8;
typedef int int32;
typedef long long int64;

typedef long long integer_t;
typedef unsigned long long uinteger_t;

const bool ASSERT_FLAG = true;

// when CHECK_RESOURCE_DEADLOCK is enabled, slicc will generate additional code
// that works in conjuction with the resources rank value specified in the protocol
// to detect invalid resource stalls as soon as they occur.
const bool CHECK_INVALID_RESOURCE_STALLS = false;

#undef assert
#define assert(EXPR) ASSERT(EXPR)

#define ASSERT(EXPR)\
{\
  if (ASSERT_FLAG) {\
    if (!(EXPR)) {\
      cerr << "failed assertion '"\
           << #EXPR << "' at fn "\
           << __PRETTY_FUNCTION__ << " in "\
           << __FILE__ << ":"\
           << __LINE__ << endl;\
      if(isatty(STDIN_FILENO)) {\
        cerr << "At this point you might want to attach a debug to ";\
        cerr << "the running and get to the" << endl;\
        cerr << "crash site; otherwise press enter to continue" << endl;\
        cerr << "PID: " << getpid();\
        cerr << endl << flush; \
        char c; \
        cin.get(c); \
      }\
      abort();\
    }\
  }\
}

class State;
class Event;
class Symbol;
class Var;

namespace __gnu_cxx {
  template <> struct hash<State*>
  {
    size_t operator()(State* s) const { return (size_t) s; }
  };
  template <> struct hash<Event*>
  {
    size_t operator()(Event* s) const { return (size_t) s; }
  };
  template <> struct hash<Symbol*>
  {
    size_t operator()(Symbol* s) const { return (size_t) s; }
  };
  template <> struct hash<Var*>
  {
    size_t operator()(Var* s) const { return (size_t) s; }
  };
} // namespace __gnu_cxx

namespace std {
  template <> struct equal_to<Event*>
  {
    bool operator()(Event* s1, Event* s2) const { return s1 == s2; }
  };
  template <> struct equal_to<State*>
  {
    bool operator()(State* s1, State* s2) const { return s1 == s2; }
  };
  template <> struct equal_to<Symbol*>
  {
    bool operator()(Symbol* s1, Symbol* s2) const { return s1 == s2; }
  };
  template <> struct equal_to<Var*>
  {
    bool operator()(Var* s1, Var* s2) const { return s1 == s2; }
  };
} // namespace std

#endif //SLICC_GLOBAL_H
