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

#ifndef __ARCH_X86_BIOS_SMBIOS_HH__
#define __ARCH_X86_BIOS_SMBIOS_HH__

#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"
#include "enums/Characteristic.hh"
#include "enums/ExtCharacteristic.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PortProxy;
struct X86SMBiosBiosInformationParams;
struct X86SMBiosSMBiosStructureParams;
struct X86SMBiosSMBiosTableParams;

namespace X86ISA
{

namespace smbios
{

class SMBiosStructure : public SimObject
{
  protected:
    typedef X86SMBiosSMBiosStructureParams Params;

  public:

    virtual
    ~SMBiosStructure()
    {}

    // Offset 00h, 1 byte
    uint8_t type;

    // Offset 01h, 1 byte
    //Length: computed when written to memory.

    // Offset 02h, 2 bytes
    uint16_t handle;

    virtual uint8_t
    getLength()
    {
        // This is the size of a structure with nothing but the header
        return 4;
    }

    virtual uint16_t writeOut(PortProxy& proxy, Addr addr);

  protected:
    bool stringFields;

    SMBiosStructure(const Params &p, uint8_t _type);

    std::vector<std::string> strings;

    void writeOutStrings(PortProxy& proxy, Addr addr);

    int getStringLength();

  public:

    int addString(const std::string &new_string);
    std::string readString(int n);
    void setString(int n, const std::string &new_string);
};

class BiosInformation : public SMBiosStructure
{
  protected:
    const static uint8_t Type = 0;

    typedef X86SMBiosBiosInformationParams Params;

  public:
    // Offset 04h, 1 byte
    uint8_t vendor;
    // Offset 05h, 1 byte
    uint8_t version;
    // Offset 06h, 2 bytes
    uint16_t startingAddrSegment;
    // Offset 08h, 1 byte
    uint8_t releaseDate;
    // Offset 09h, 1 byte
    uint8_t romSize;
    // Offset 0Ah, 8 bytes
    //See tables in 3.3.1 in the SMBios 2.5 spec from the DMTF for
    //bit definitions.
    uint64_t characteristics;
    // Offset 12h, 2 bytes
    uint16_t characteristicExtBytes;
    // Offset 14h, 1 byte
    uint8_t majorVer;
    // Offset 15h, 1 byte
    uint8_t minorVer;
    // Offset 16h, 1 byte
    uint8_t embContFirmwareMajor;
    // Offset 17h, 1 byte
    uint8_t embContFirmwareMinor;

    BiosInformation(const Params &p);

    uint8_t getLength() { return 0x18; }
    uint16_t writeOut(PortProxy& proxy, Addr addr);
};

class SMBiosTable : public SimObject
{
  protected:
    typedef X86SMBiosSMBiosTableParams Params;

    struct SMBiosHeader
    {
        SMBiosHeader()
        {}

        // Offset 00h, 4 bytes
        static const char anchorString[];

        // Offset 04h, 1 byte
        //Checksum: computed when written to memory.

        // Offset 05h, 1 byte
        static const uint8_t entryPointLength;

        // Offset 06h, 1 byte
        uint8_t majorVersion;

        // Offset 07h, 1 byte
        uint8_t minorVersion;

        // Offset 08h, 2 bytes
        //Maximum structure size: computed when written to memory.

        // Offset 0Ah, 1 byte
        static const uint8_t entryPointRevision;

        // Offset 0Bh, 5 bytes
        static const uint8_t formattedArea[5];

        // Offset 10h, 15 bytes
        struct IntermediateHeader
        {
            IntermediateHeader() : tableAddr(0)
            {}
            // Offset 10h, 5 bytes
            static const char anchorString[];

            // Offset 15h, 1 byte
            //Checksum: computed when written to memory.

            // Offset 16h, 2 bytes
            //Length of the structure table in bytes: computed when
            //written to memory.

            // Offset 18h, 4 bytes
            uint32_t tableAddr;

            // Offset 1Ch, 2 bytes
            //Number of structures: computed when written to memory

            // Offset 1Eh, 1 byte
            uint8_t smbiosBCDRevision;
        } intermediateHeader;
    } smbiosHeader;

    std::vector<SMBiosStructure *> structures;

  public:
    SMBiosTable(const Params &p);

    Addr getTableAddr()
    {
        return smbiosHeader.intermediateHeader.tableAddr;
    }

    void setTableAddr(Addr addr)
    {
        smbiosHeader.intermediateHeader.tableAddr = addr;
    }

    void writeOut(PortProxy& proxy, Addr addr,
            Addr &headerSize, Addr &structSize);
};

} // namespace smbios
} // namespace X86ISA
} // namespace gem5

#endif
