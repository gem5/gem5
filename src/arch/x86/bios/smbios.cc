/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#include "arch/x86/bios/smbios.hh"

#include "base/types.hh"
#include "mem/port_proxy.hh"
#include "params/X86SMBiosBiosInformation.hh"
#include "params/X86SMBiosSMBiosStructure.hh"
#include "params/X86SMBiosSMBiosTable.hh"
#include "sim/byteswap.hh"

namespace gem5
{

const char X86ISA::smbios::SMBiosTable::SMBiosHeader::anchorString[] = "_SM_";
const uint8_t X86ISA::smbios::SMBiosTable::SMBiosHeader::formattedArea[] = {
    0, 0, 0, 0, 0
};
const uint8_t X86ISA::smbios::SMBiosTable::SMBiosHeader::entryPointLength =
    0x1F;
const uint8_t X86ISA::smbios::SMBiosTable::SMBiosHeader::entryPointRevision =
    0;
const char X86ISA::smbios::SMBiosTable::SMBiosHeader::IntermediateHeader::
    anchorString[] = "_DMI_";

template <class T>
uint64_t
composeBitVector(T vec)
{
    uint64_t val = 0;
    typename T::iterator vecIt;
    for (vecIt = vec.begin(); vecIt != vec.end(); vecIt++) {
        val |= (1 << (*vecIt));
    }
    return val;
}

uint16_t
X86ISA::smbios::SMBiosStructure::writeOut(PortProxy &proxy, Addr addr)
{
    proxy.writeBlob(addr, &type, 1);

    uint8_t length = getLength();
    proxy.writeBlob(addr + 1, &length, 1);

    uint16_t handleGuest = htole(handle);
    proxy.writeBlob(addr + 2, &handleGuest, 2);

    return length + getStringLength();
}

X86ISA::smbios::SMBiosStructure::SMBiosStructure(const Params &p,
                                                 uint8_t _type)
    : SimObject(p), type(_type), handle(0), stringFields(false)
{}

void
X86ISA::smbios::SMBiosStructure::writeOutStrings(PortProxy &proxy, Addr addr)
{
    std::vector<std::string>::iterator it;
    Addr offset = 0;

    const uint8_t nullTerminator = 0;

    // If there are string fields but none of them are used, that's a
    // special case which is handled by this if.
    if (strings.size() == 0 && stringFields) {
        proxy.writeBlob(addr + offset, &nullTerminator, 1);
        offset++;
    } else {
        for (it = strings.begin(); it != strings.end(); it++) {
            proxy.writeBlob(addr + offset, it->c_str(), it->length() + 1);
            offset += it->length() + 1;
        }
    }
    proxy.writeBlob(addr + offset, &nullTerminator, 1);
}

int
X86ISA::smbios::SMBiosStructure::getStringLength()
{
    int size = 0;
    std::vector<std::string>::iterator it;

    for (it = strings.begin(); it != strings.end(); it++) {
        size += it->length() + 1;
    }

    return size + 1;
}

int
X86ISA::smbios::SMBiosStructure::addString(const std::string &new_string)
{
    stringFields = true;
    // If a string is empty, treat it as not existing. The index for empty
    // strings is 0.
    if (new_string.length() == 0)
        return 0;
    strings.push_back(new_string);
    return strings.size();
}

std::string
X86ISA::smbios::SMBiosStructure::readString(int n)
{
    assert(n > 0 && n <= strings.size());
    return strings[n - 1];
}

void
X86ISA::smbios::SMBiosStructure::setString(int n,
                                           const std::string &new_string)
{
    assert(n > 0 && n <= strings.size());
    strings[n - 1] = new_string;
}

X86ISA::smbios::BiosInformation::BiosInformation(const Params &p)
    : SMBiosStructure(p, Type),
      startingAddrSegment(p.starting_addr_segment),
      romSize(p.rom_size),
      majorVer(p.major),
      minorVer(p.minor),
      embContFirmwareMajor(p.emb_cont_firmware_major),
      embContFirmwareMinor(p.emb_cont_firmware_minor)
{
    vendor = addString(p.vendor);
    version = addString(p.version);
    releaseDate = addString(p.release_date);

    characteristics = composeBitVector(p.characteristics);
    characteristicExtBytes = composeBitVector(p.characteristic_ext_bytes);
}

uint16_t
X86ISA::smbios::BiosInformation::writeOut(PortProxy &proxy, Addr addr)
{
    uint8_t size = SMBiosStructure::writeOut(proxy, addr);

    proxy.writeBlob(addr + 0x4, &vendor, 1);
    proxy.writeBlob(addr + 0x5, &version, 1);

    uint16_t startingAddrSegmentGuest = htole(startingAddrSegment);
    proxy.writeBlob(addr + 0x6, &startingAddrSegmentGuest, 2);

    proxy.writeBlob(addr + 0x8, &releaseDate, 1);
    proxy.writeBlob(addr + 0x9, &romSize, 1);

    uint64_t characteristicsGuest = htole(characteristics);
    proxy.writeBlob(addr + 0xA, &characteristicsGuest, 8);

    uint16_t characteristicExtBytesGuest = htole(characteristicExtBytes);
    proxy.writeBlob(addr + 0x12, &characteristicExtBytesGuest, 2);

    proxy.writeBlob(addr + 0x14, &majorVer, 1);
    proxy.writeBlob(addr + 0x15, &minorVer, 1);
    proxy.writeBlob(addr + 0x16, &embContFirmwareMajor, 1);
    proxy.writeBlob(addr + 0x17, &embContFirmwareMinor, 1);

    writeOutStrings(proxy, addr + getLength());

    return size;
}

X86ISA::smbios::SMBiosTable::SMBiosTable(const Params &p)
    : SimObject(p), structures(p.structures)
{
    smbiosHeader.majorVersion = p.major_version;
    smbiosHeader.minorVersion = p.minor_version;
    assert(p.major_version <= 9);
    assert(p.minor_version <= 9);
    smbiosHeader.intermediateHeader.smbiosBCDRevision =
        (p.major_version << 4) | p.minor_version;
}

void
X86ISA::smbios::SMBiosTable::writeOut(PortProxy &proxy, Addr addr,
                                      Addr &headerSize, Addr &structSize)
{
    headerSize = 0x1F;

    /*
     * The main header
     */
    uint8_t mainChecksum = 0;

    proxy.writeBlob(addr, (uint8_t *)smbiosHeader.anchorString, 4);
    for (int i = 0; i < 4; i++)
        mainChecksum += smbiosHeader.anchorString[i];

    // The checksum goes here, but we're figuring it out as we go.

    proxy.writeBlob(addr + 0x5, &smbiosHeader.entryPointLength, 1);
    mainChecksum += smbiosHeader.entryPointLength;
    proxy.writeBlob(addr + 0x6, &smbiosHeader.majorVersion, 1);
    mainChecksum += smbiosHeader.majorVersion;
    proxy.writeBlob(addr + 0x7, &smbiosHeader.minorVersion, 1);
    mainChecksum += smbiosHeader.minorVersion;
    // Maximum structure size goes here, but we'll figure it out later.
    proxy.writeBlob(addr + 0xA, &smbiosHeader.entryPointRevision, 1);
    mainChecksum += smbiosHeader.entryPointRevision;
    proxy.writeBlob(addr + 0xB, &smbiosHeader.formattedArea, 5);
    for (int i = 0; i < 5; i++)
        mainChecksum += smbiosHeader.formattedArea[i];

    /*
     * The intermediate header
     */
    uint8_t intChecksum = 0;

    proxy.writeBlob(addr + 0x10, smbiosHeader.intermediateHeader.anchorString,
                    5);
    for (int i = 0; i < 5; i++)
        intChecksum += smbiosHeader.intermediateHeader.anchorString[i];

    // The checksum goes here, but we're figuring it out as we go.
    // Then the length of the structure table which we'll find later

    uint32_t tableAddrGuest = htole(smbiosHeader.intermediateHeader.tableAddr);
    proxy.writeBlob(addr + 0x18, &tableAddrGuest, 4);
    for (int i = 0; i < 4; i++) {
        intChecksum += tableAddrGuest;
        tableAddrGuest >>= 8;
    }

    uint16_t numStructs = letoh(structures.size());
    proxy.writeBlob(addr + 0x1C, &numStructs, 2);
    for (int i = 0; i < 2; i++) {
        intChecksum += numStructs;
        numStructs >>= 8;
    }

    proxy.writeBlob(addr + 0x1E,
                    &smbiosHeader.intermediateHeader.smbiosBCDRevision, 1);
    intChecksum += smbiosHeader.intermediateHeader.smbiosBCDRevision;

    /*
     * Structure table
     */

    Addr base = smbiosHeader.intermediateHeader.tableAddr;
    Addr offset = 0;
    uint16_t maxSize = 0;
    std::vector<SMBiosStructure *>::iterator it;
    for (it = structures.begin(); it != structures.end(); it++) {
        uint16_t size = (*it)->writeOut(proxy, base + offset);
        if (size > maxSize)
            maxSize = size;
        offset += size;
    }

    structSize = offset;

    /*
     * Header
     */

    maxSize = htole(maxSize);
    proxy.writeBlob(addr + 0x8, &maxSize, 2);
    for (int i = 0; i < 2; i++) {
        mainChecksum += maxSize;
        maxSize >>= 8;
    }

    // Set the checksum
    mainChecksum = -mainChecksum;
    proxy.writeBlob(addr + 0x4, &mainChecksum, 1);

    /*
     * Intermediate header
     */

    uint16_t tableSize = offset;
    tableSize = htole(tableSize);
    proxy.writeBlob(addr + 0x16, &tableSize, 2);
    for (int i = 0; i < 2; i++) {
        intChecksum += tableSize;
        tableSize >>= 8;
    }

    intChecksum = -intChecksum;
    proxy.writeBlob(addr + 0x15, &intChecksum, 1);
}

} // namespace gem5
