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

/**
 * @file
 * Declaration of top level class for the Tsunami chipset. This class just
 * retains pointers to all its children so the children can communicate.
 */

#ifndef __TSUNAMI_HH__
#define __TSUNAMI_HH__

#include "sim/sim_object.hh"

class IntrControl;
class ConsoleListener;
class SimConsole;
class AdaptecController;
class TlaserClock;
class EtherDev;
class TsunamiCChip;
class TsunamiPChip;
class PCIConfigAll;

/**
  * Top level class for Tsunami Chipset emulation.
  * This structure just contains pointers to all the
  * children so the children can commnicate to do the
  * read work
  */

class Tsunami : public SimObject
{
  public:

    /** Max number of CPUs in a Tsunami */
    static const int Max_CPUs = 4;

    /** Pointer to the interrupt controller (used to post and ack interrupts on the CPU) */
    IntrControl *intrctrl;
    /** Pointer to the UART emulation code */
    SimConsole *cons;

    /** Pointer to the SCSI controller device */
    AdaptecController *scsi;
    /** Pointer to the ethernet controller device */
    EtherDev *ethernet;

    /** Pointer to the Tsunami CChip.
      * The chip contains some configuration information and
      * all the interrupt mask and status registers
      */
    TsunamiCChip *cchip;

    /** Pointer to the Tsunami PChip.
      * The pchip is the interface to the PCI bus, in our case
      * it does not have to do much.
      */
    TsunamiPChip *pchip;

    /** Pointer to the Tsunami PCI Config Space
      * The config space in tsunami all needs to return
      * -1 if a device is not there.
      */
    PCIConfigAll *pciconfig;

    int intr_sum_type[Tsunami::Max_CPUs];
    int ipi_pending[Tsunami::Max_CPUs];

    int interrupt_frequency;

  public:
    /**
      * Constructor for the Tsunami Class.
      * @param name name of the object
      * @param scsi pointer to scsi controller object
      * @param con pointer to the console
      * @param intrcontrol pointer to the interrupt controller
      * @param intrFreq frequency that interrupts happen
      */
    Tsunami(const std::string &name, AdaptecController *scsi,
               EtherDev *ethernet,
               SimConsole *con, IntrControl *intctrl, int intrFreq);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __TSUNAMI_HH__
