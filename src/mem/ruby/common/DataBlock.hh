
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

#ifndef DATABLOCK_H
#define DATABLOCK_H

#include "Global.hh"
#include "RubyConfig.hh"
#include "Vector.hh"

class DataBlock {
public:
  // Constructors
  DataBlock();

  // Destructor
  ~DataBlock();

  // Public Methods
  void clear();
  uint8 getByte(int whichByte) const;
  void setByte(int whichByte, uint8 data);
  bool equal(const DataBlock& obj) const;
  void print(ostream& out) const;

private:
  // Private Methods

  // Data Members (m_ prefix)
  Vector<uint8> m_data;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DataBlock& obj);

bool operator==(const DataBlock& obj1, const DataBlock& obj2);


// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const DataBlock& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

extern inline
bool operator==(const DataBlock& obj1,const DataBlock& obj2)
{
  return (obj1.equal(obj2));
}

#endif //DATABLOCK_H
