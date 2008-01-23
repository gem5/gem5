/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 * Authors: Gabe Black
 */

#include "arch/x86/smbios.hh"
#include "arch/x86/isa_traits.hh"
#include "mem/port.hh"
#include "sim/byteswap.hh"
#include "sim/host.hh"

const char X86ISA::SMBios::SMBiosTable::SMBiosHeader::anchorString[] = "_SM_";
const uint8_t X86ISA::SMBios::SMBiosTable::
        SMBiosHeader::formattedArea[] = {0,0,0,0,0};
const uint8_t X86ISA::SMBios::SMBiosTable::
        SMBiosHeader::entryPointLength = 0x1F;
const uint8_t X86ISA::SMBios::SMBiosTable::
        SMBiosHeader::entryPointRevision = 0;
const char X86ISA::SMBios::SMBiosTable::
        SMBiosHeader::IntermediateHeader::anchorString[] = "_DMI_";

uint16_t
X86ISA::SMBios::BiosInformation::writeOut(FunctionalPort * port, Addr addr)
{
    uint8_t size = SMBiosStructure::writeOut(port, addr);

    port->writeBlob(addr + 0x4, (uint8_t *)(&vendor), 1);
    port->writeBlob(addr + 0x5, (uint8_t *)(&version), 1);

    uint16_t startingAddrSegmentGuest = X86ISA::htog(startingAddrSegment);
    port->writeBlob(addr + 0x6, (uint8_t *)(&startingAddrSegmentGuest), 2);

    port->writeBlob(addr + 0x8, (uint8_t *)(&releaseDate), 1);
    port->writeBlob(addr + 0x9, (uint8_t *)(&romSize), 1);

    uint64_t characteristicsGuest = X86ISA::htog(characteristics);
    port->writeBlob(addr + 0xA, (uint8_t *)(&characteristicsGuest), 8);

    uint16_t characteristicExtBytesGuest =
        X86ISA::htog(characteristicExtBytes);
    port->writeBlob(addr + 0x12, (uint8_t *)(&characteristicExtBytesGuest), 2);

    port->writeBlob(addr + 0x14, (uint8_t *)(&major), 1);
    port->writeBlob(addr + 0x15, (uint8_t *)(&minor), 1);
    port->writeBlob(addr + 0x16, (uint8_t *)(&embContFirmwareMajor), 1);
    port->writeBlob(addr + 0x17, (uint8_t *)(&embContFirmwareMinor), 1);

    writeOutStrings(port, addr + getLength());

    return size;
}

void
X86ISA::SMBios::SMBiosTable::writeOut(FunctionalPort * port, Addr addr)
{

    /*
     * The main header
     */
    uint8_t mainChecksum = 0;

    port->writeBlob(addr, (uint8_t *)smbiosHeader.anchorString, 4);
    for (int i = 0; i < 4; i++)
        mainChecksum += smbiosHeader.anchorString[i];

    // The checksum goes here, but we're figuring it out as we go.

    port->writeBlob(addr + 0x5,
            (uint8_t *)(&smbiosHeader.entryPointLength), 1);
    mainChecksum += smbiosHeader.entryPointLength;
    port->writeBlob(addr + 0x6,
            (uint8_t *)(&smbiosHeader.majorVersion), 1);
    mainChecksum += smbiosHeader.majorVersion;
    port->writeBlob(addr + 0x7,
            (uint8_t *)(&smbiosHeader.minorVersion), 1);
    mainChecksum += smbiosHeader.minorVersion;
    // Maximum structure size goes here, but we'll figure it out later.
    port->writeBlob(addr + 0xA,
            (uint8_t *)(&smbiosHeader.entryPointRevision), 1);
    mainChecksum += smbiosHeader.entryPointRevision;
    port->writeBlob(addr + 0xB,
            (uint8_t *)(&smbiosHeader.formattedArea), 5);
    for (int i = 0; i < 5; i++)
        mainChecksum += smbiosHeader.formattedArea[i];

    /*
     * The intermediate header
     */
    uint8_t intChecksum = 0;

    port->writeBlob(addr + 0x10,
            (uint8_t *)smbiosHeader.intermediateHeader.anchorString, 5);
    for (int i = 0; i < 5; i++)
        intChecksum += smbiosHeader.intermediateHeader.anchorString[i];

    // The checksum goes here, but we're figuring it out as we go.
    // Then the length of the structure table which we'll find later

    uint32_t tableAddrGuest =
        X86ISA::htog(smbiosHeader.intermediateHeader.tableAddr);
    port->writeBlob(addr + 0x18, (uint8_t *)(&tableAddrGuest), 4);
    for (int i = 0; i < 4; i++) {
        intChecksum += tableAddrGuest;
        tableAddrGuest >>= 8;
    }

    uint16_t numStructs = X86ISA::gtoh(structures.size());
    port->writeBlob(addr + 0x1C, (uint8_t *)(&numStructs), 2);
    for (int i = 0; i < 2; i++) {
        intChecksum += numStructs;
        numStructs >>= 8;
    }

    port->writeBlob(addr + 0x1E,
            (uint8_t *)(&smbiosHeader.intermediateHeader.smbiosBCDRevision),
            1);
    intChecksum += smbiosHeader.intermediateHeader.smbiosBCDRevision;

    /*
     * Structure table
     */

    Addr base = smbiosHeader.intermediateHeader.tableAddr;
    Addr offset = 0;
    uint16_t maxSize = 0;
    std::vector<SMBiosStructure>::iterator it;
    for (it = structures.begin(); it != structures.end(); it++) {
        uint16_t size = it->writeOut(port, base + offset);
        if (size > maxSize)
            maxSize = size;
        offset += size;
    }

    /*
     * Header
     */

    maxSize = X86ISA::htog(maxSize);
    port->writeBlob(addr + 0x8, (uint8_t *)(&maxSize), 2);
    for (int i = 0; i < 2; i++) {
        mainChecksum += maxSize;
        maxSize >>= 8;
    }

    // Set the checksum
    mainChecksum = -mainChecksum;
    port->writeBlob(addr + 0x4, (uint8_t *)(&mainChecksum), 1);

    /*
     * Intermediate header
     */

    uint16_t tableSize = offset;
    tableSize = X86ISA::htog(tableSize);
    port->writeBlob(addr + 0x16, (uint8_t *)(&tableSize), 2);
    for (int i = 0; i < 2; i++) {
        intChecksum += tableSize;
        tableSize >>= 8;
    }

    intChecksum = -intChecksum;
    port->writeBlob(addr + 0x15, (uint8_t *)(&intChecksum), 1);
}
