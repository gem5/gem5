/*
 * Copyright (c) 2019 The Regents of the University of California
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/* @file
 * User Console Definitions
 */

#ifndef __BASE_MATCH_HH__
#define __BASE_MATCH_HH__

#include <string>
#include <vector>

/**
 * ObjectMatch contains a vector of expressions. ObjectMatch can then be
 * queried, via ObjectMatch.match(std::string), to check if a string matches
 * any expressions in the vector.
 *
 * Expressions in ObjectMatch take the form "<token1>.<token2>.<token3>"; a
 * series of expected tokens separated by a period. The input string takes the
 * form "<value1>.<value2>.<value3>". In this case, the input string matches
 * the expression if <value1> == <token1> && <token2> == <value2>
 * && <value3> == <token3>.  A token may be a wildcard character, "*", which
 * will match to any value in that position (inclusive of no value at that
 * location).
 */
class ObjectMatch
{
  protected:
    std::vector<std::vector<std::string> > tokens;
    bool domatch(const std::string &name) const;

  public:
    ObjectMatch();
    ObjectMatch(const std::string &expression);
    void add(const ObjectMatch &other);
    void setExpression(const std::string &expression);
    void setExpression(const std::vector<std::string> &expression);
    std::vector<std::vector<std::string> > getExpressions();
    bool match(const std::string &name) const
    {
        return tokens.empty() ? false : domatch(name);
    }
};

#endif // __BASE_MATCH_HH__

