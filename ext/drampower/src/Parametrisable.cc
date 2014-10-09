/*
 * Copyright (c) 2012-2014, TU Delft
 * Copyright (c) 2012-2014, TU Eindhoven
 * Copyright (c) 2012-2014, TU Kaiserslautern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Hansson
 *
 */

#include "Parametrisable.h"

#include <iostream>
#include <cstdlib>

using namespace Data;
using namespace std;

void Parametrisable::pushParameter(const Parameter& parameter)
{
  parameters.push_back(parameter);
}

void Parametrisable::setParameter(const Parameter& parameter,
                                  unsigned int     index)
{
  unsigned int count            = 0;

  vector<Parameter>::iterator p = parameters.begin();

  while (p != parameters.end() && !(p->getId() == parameter.getId() &&
                                    index == count)) {
    if (p->getId() == parameter.getId())
      ++count;
    ++p;
  }

  if (p == parameters.end()) {
    parameters.push_back(parameter);
  } else {
    p = parameters.erase(p);
    parameters.insert(p, parameter);
  }
} // Parametrisable::setParameter

bool Parametrisable::removeParameter(const string& id, unsigned int index)
{
  unsigned int count = 0;

  for (vector<Parameter>::iterator p = parameters.begin();
       p != parameters.end(); ++p) {
    if ((p->getId() == id) && (index == count++)) {
      parameters.erase(p);
      return true;
    }
  }

  return false;
}

/**
 * Get a parameter with a specific id. Should there be a multiplicity,
 * then the index is used to determine which instance is returned, in
 * order traversal.
 */
Parameter Parametrisable::getParameter(const string& id,
                                       unsigned int  index) const
{
  unsigned int count = 0;

  for (vector<Parameter>::const_iterator p = parameters.begin();
       p != parameters.end(); ++p) {
    if ((p->getId() == id) && (index == count++)) {
      return *p;
    }
  }

  cerr << "Could not find parameter '" << id << "' (" << index << ")" << endl;
  cerr << "Stored parameters are: " << endl;
  for (vector<Parameter>::const_iterator p = parameters.begin();
       p != parameters.end(); ++p) {
    cerr << "   " << p->getId() << ": " << p->getValue() << endl;
  }
  exit(1);

  return Parameter("", "", "");
} // Parametrisable::getParameter

vector<Parameter> Parametrisable::getParameters() const
{
  return parameters;
}

bool Parametrisable::hasParameter(const string& id, unsigned int index) const
{
  unsigned int count = 0;

  for (vector<Parameter>::const_iterator p = parameters.begin();
       p != parameters.end(); ++p) {
    if ((p->getId() == id) && (index == count++)) {
      return true;
    }
  }

  return false;
}
