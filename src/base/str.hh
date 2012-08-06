/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __STR_HH__
#define __STR_HH__

#include <cctype>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

template<class> class Hash;
template<>
class Hash<std::string> {
public:
  unsigned operator()(const std::string &s) {
      std::string::const_iterator i = s.begin();
      std::string::const_iterator end = s.end();
      unsigned hash = 5381;

      while (i < end)
          hash = ((hash << 5) + hash) + *i++;

      return hash;
  }
};

inline void
eat_lead_white(std::string &s)
{
    std::string::size_type off = s.find_first_not_of(' ');
    if (off != std::string::npos) {
        std::string::iterator begin = s.begin();
        s.erase(begin, begin + off);
    }
}

inline void
eat_end_white(std::string &s)
{
    std::string::size_type off = s.find_last_not_of(' ');
    if (off != std::string::npos)
        s.erase(s.begin() + off + 1, s.end());
}

inline void
eat_white(std::string &s)
{
    eat_lead_white(s);
    eat_end_white(s);
}

inline std::string
to_lower(const std::string &s)
{
    std::string lower;
    int len = s.size();

    lower.reserve(len);

    for (int i = 0; i < len; ++i)
        lower.push_back(tolower(s[i]));

    return lower;
}

// Split the string s into lhs and rhs on the first occurence of the
// character c.
bool
split_first(const std::string &s, std::string &lhs, std::string &rhs, char c);

// Split the string s into lhs and rhs on the last occurence of the
// character c.
bool
split_last(const std::string &s, std::string &lhs, std::string &rhs, char c);

// Tokenize the string <s> splitting on the character <token>, and
// place the result in the string vector <vector>.  If <ign> is true,
// then empty result strings (due to trailing tokens, or consecutive
// tokens) are skipped.
void
tokenize(std::vector<std::string> &vector, const std::string &s,
         char token, bool ign = true);

template <class T> bool
to_number(const std::string &value, T &retval);

template <class T>
inline std::string
to_string(const T &value)
{
    std::stringstream str;
    str << value;
    return str.str();
}

// Put quotes around string arg if it contains spaces.
inline std::string
quote(const std::string &s)
{
    std::string ret;
    bool quote = s.find(' ') != std::string::npos;

    if (quote)
        ret = '"';

    ret += s;

    if (quote)
        ret += '"';

    return ret;
}


/**
 * Return true if 's' starts with the prefix string 'prefix'.
 */
inline bool
startswith(const char *s, const char *prefix)
{
    return (strncmp(s, prefix, strlen(prefix)) == 0);
}


/**
 * Return true if 's' starts with the prefix string 'prefix'.
 */
inline bool
startswith(const std::string &s, const char *prefix)
{
    return (s.compare(0, strlen(prefix), prefix) == 0);
}


/**
 * Return true if 's' starts with the prefix string 'prefix'.
 */
inline bool
startswith(const std::string &s, const std::string &prefix)
{
    return (s.compare(0, prefix.size(), prefix) == 0);
}


#endif //__STR_HH__
