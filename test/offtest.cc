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

#include <sys/types.h>
#include <stddef.h>
#include <stdio.h>
#include "dev/pcireg.h"

int
main()
{
#define POFFSET(x) \
  printf("offsetof(PCIConfig, hdr."#x") = %d\n", \
          offsetof(PCIConfig, hdr.x))

  POFFSET(vendor);
  POFFSET(device);
  POFFSET(command);
  POFFSET(status);
  POFFSET(revision);
  POFFSET(progIF);
  POFFSET(subClassCode);
  POFFSET(classCode);
  POFFSET(cacheLineSize);
  POFFSET(latencyTimer);
  POFFSET(headerType);
  POFFSET(bist);
  POFFSET(pci0.baseAddr0);
  POFFSET(pci0.baseAddr1);
  POFFSET(pci0.baseAddr2);
  POFFSET(pci0.baseAddr3);
  POFFSET(pci0.baseAddr4);
  POFFSET(pci0.baseAddr5);
  POFFSET(pci0.cardbusCIS);
  POFFSET(pci0.subsystemVendorID);
  POFFSET(pci0.expansionROM);
  POFFSET(pci0.reserved0);
  POFFSET(pci0.reserved1);
  POFFSET(pci0.interruptLine);
  POFFSET(pci0.interruptPin);
  POFFSET(pci0.minimumGrant);
  POFFSET(pci0.minimumLatency);

  return 0;
}
