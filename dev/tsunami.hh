/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __TSUNAMI_HH__
#define __TSUNAMI_HH__

#include "sim/sim_object.hh"

class IntrControl;
class ConsoleListener;
class SimConsole;
class ScsiController;
class TlaserClock;
class EtherDev;
class TsunamiCChip;
class TsunamiPChip;

class Tsunami : public SimObject
{
  public:

    static const int Max_CPUs = 4;

    IntrControl *intrctrl;
//    ConsoleListener *listener;
    SimConsole *cons;

    ScsiController *scsi;
    EtherDev *ethernet;

    TsunamiCChip *cchip;
    TsunamiPChip *pchip;

    int intr_sum_type[Tsunami::Max_CPUs];
    int ipi_pending[Tsunami::Max_CPUs];

    int interrupt_frequency;

  public:
    Tsunami(const std::string &name, ScsiController *scsi,
               EtherDev *ethernet,
               SimConsole *, IntrControl *intctrl, int intrFreq);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __TSUNAMI_HH__
