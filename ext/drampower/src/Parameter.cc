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

#include "Parameter.h"

#include <iomanip>
#include "Utils.h"

using namespace Data;
using namespace std;

Parameter::Parameter(const string& id, const string& type,
                     const string& value) : id(id), type(type), value(value)
{
}

string Parameter::getId() const
{
  return id;
}

string Parameter::getType() const
{
  return type;
}

int Parameter::getIntValue() const
{
  return fromString<int>(value);
}

unsigned int Parameter::getUIntValue() const
{
  bool isHex = value.size() > 1 && value[0] == '0' && value[1] == 'x';

  return fromString<unsigned int>(value, isHex ? std::hex : std::dec);
}

#ifdef _LP64

size_t Parameter::getSizeTValue() const
{
  bool isHex = value.size() > 1 && value[0] == '0' && value[1] == 'x';

  return fromString<size_t>(value, isHex ? std::hex : std::dec);
}

#endif

double Parameter::getDoubleValue() const
{
  return fromString<double>(value);
}

bool Parameter::getBoolValue() const
{
  return fromString<bool>(value);
}

string Parameter::getValue() const
{
  return value;
}

Parameter Data::HexParameter(const string& id, int value)
{
  std::ostringstream ss;

  ss << "0x" << hex << setw(8) << setfill('0') << value;

  return Parameter(id, "int", ss.str());
}

Parameter Data::StringParameter(const string& id, const string& value)
{
  return Parameter(id, "string", value);
}

ostream& Data::operator<<(ostream& os, const Parameter& parameter)
{
  os << "<parameter " <<
    "id=\"" << parameter.getId() << "\" " <<
    "type=\"" << parameter.getType() << "\" "
                                      "value=\"" << parameter.getValue() << "\" />";

  return os;
}
