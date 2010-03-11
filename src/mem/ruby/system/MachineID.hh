
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
 * NodeID.hh
 *
 * Description:
 *
 * $Id$
 *
 */

#ifndef MACHINEID_H
#define MACHINEID_H

#include <iostream>
#include <string>

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/util.hh"
#include "mem/protocol/MachineType.hh"

struct MachineID {
  MachineType type;
  int num;  // range: 0 ... number of this machine's components in the system - 1
};

extern inline
std::string MachineIDToString (MachineID machine) {
  return MachineType_to_string(machine.type)+"_"+int_to_string(machine.num);
}

extern inline
bool operator==(const MachineID & obj1, const MachineID & obj2)
{
  return (obj1.type == obj2.type && obj1.num == obj2.num);
}

extern inline
bool operator!=(const MachineID & obj1, const MachineID & obj2)
{
  return (obj1.type != obj2.type || obj1.num != obj2.num);
}

// Output operator declaration
std::ostream& operator<<(std::ostream& out, const MachineID& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
std::ostream& operator<<(std::ostream& out, const MachineID& obj)
{
  if ((obj.type < MachineType_NUM) && (obj.type >= MachineType_FIRST)) {
    out << MachineType_to_string(obj.type);
  } else {
    out << "NULL";
  }
  out << "-";
  out << obj.num;
  out << std::flush;
  return out;
}


#endif //MACHINEID_H
