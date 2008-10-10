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

#ifndef __ARCH_X86_BIOS_SMBIOS_HH__
#define __ARCH_X86_BIOS_SMBIOS_HH__

#include <string>
#include <vector>

#include "arch/x86/isa_traits.hh"
#include "mem/port.hh"
#include "sim/byteswap.hh"
#include "sim/host.hh"

namespace X86ISA
{

namespace SMBios
{

class SMBiosStructure
{
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

    virtual uint16_t
    writeOut(FunctionalPort * port, Addr addr)
    {
        port->writeBlob(addr, (uint8_t *)(&type), 1);

        uint8_t length = getLength();
        port->writeBlob(addr + 1, (uint8_t *)(&length), 1);

        uint16_t handleGuest = X86ISA::htog(handle);
        port->writeBlob(addr + 2, (uint8_t *)(&handleGuest), 2);

        return length + getStringLength();
    }

  protected:
    std::vector<std::string> strings;

    void writeOutStrings(FunctionalPort * port, Addr addr)
    {
        std::vector<std::string>::iterator it;
        Addr offset = 0;

        for (it = strings.begin(); it != strings.end(); it++) {
            port->writeBlob(addr + offset,
                    (uint8_t *)it->c_str(), it->length() + 1);
            offset += it->length() + 1;
        }

        const uint8_t nullTerminator = 0;
        port->writeBlob(addr + offset, (uint8_t *)(&nullTerminator), 1);
    }

    int getStringLength()
    {
        int size = 0;
        std::vector<std::string>::iterator it;

        for (it = strings.begin(); it != strings.end(); it++) {
            size += it->length() + 1;
        }

        return size + 1;
    }

  public:

    int addString(std::string & newString)
    {
        strings.push_back(newString);
        return strings.size();
    }

    std::string readString(int n)
    {
        assert(n > 0 && n <= strings.size());
        return strings[n - 1];
    }

    void setString(int n, std::string & newString)
    {
        assert(n > 0 && n <= strings.size());
        strings[n - 1] = newString;
    }
};

class BiosInformation : public SMBiosStructure
{
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
    uint8_t major;
    // Offset 15h, 1 byte
    uint8_t minor;
    // Offset 16h, 1 byte
    uint8_t embContFirmwareMajor;
    // Offset 17h, 1 byte
    uint8_t embContFirmwareMinor;

    uint8_t getLength() { return 0x18; }
    uint16_t writeOut(FunctionalPort * port, Addr addr);
};

class SMBiosTable
{
  public:
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

    void writeOut(FunctionalPort * port, Addr addr);

    std::vector<SMBiosStructure> structures;
};

} //SMBios
} //X86ISA

#endif
