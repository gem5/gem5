/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Jaidev Patwardhan
 */

#include <cctype>
#include <cstdio>
#include <list>
#include <string>

#include "base/loader/hex_file.hh"
#include "base/loader/symtab.hh"
#include "base/cprintf.hh"
#include "mem/port_proxy.hh"

using namespace std;
/*
 * Load a Hex File into memory. Currently only used with MIPS
 * BARE_IRON mode. A hex file consists of [Address Data] tuples that
 * get directly loaded into physical memory. The address specified is
 * a word address (i.e., to get the byte address, shift left by 2) The
 * data is a full 32-bit hex value.
*/
HexFile::HexFile(const string _filename)
    : filename(_filename)
{
    fp = fopen(filename.c_str(), "r");
    if (fp == NULL)
        panic("Unable to open %s\n", filename.c_str());
}

HexFile::~HexFile()
{
    if (fp != NULL)
        fclose(fp);
}

bool
HexFile::loadSections(PortProxy& memProxy)
{
    char Line[64];
    Addr MemAddr;
    uint32_t Data;
    while (!feof(fp)) {
        char *ret = fgets(Line, sizeof(Line), fp);
        if (!ret)
            panic("malformed file");
        parseLine(Line, &MemAddr, &Data);
        if (MemAddr != 0) {
            // Now, write to memory
            memProxy.writeBlob(MemAddr << 2, (uint8_t *)&Data, sizeof(Data));
        }
    }
    return true;
}

void
HexFile::parseLine(char *Str, Addr *A, uint32_t *D)
{
    int i = 0;
    bool Flag = false;
    *A = 0;
    *D = 0;
    int Digit = 0;
    unsigned Number = 0;

    /* Skip white spaces */
    while (Str[i] != '\0' && Str[i]==' ')
        i++;

    /* Ok, we're at some character...process things */
    while (Str[i] != '\0') {
        if (Str[i] >= '0' && Str[i] <= '9') {
            Digit = Str[i] - '0';
        } else if (Str[i] >= 'a' && Str[i] <= 'f') {
            Digit = Str[i] - 'a' + 10;
        } else if (Str[i] >= 'A' && Str[i] <= 'F') {
          Digit=Str[i]-'A'+10;
        } else if (Str[i] == ' ' || Str[i] == '\n') {
            if (Number == 0)
                return;
            if (!Flag) {
                *A = Number;
                Number = 0;
                Flag = true;
            } else {
                *D = Number;
                return;
            }
        } else {
            // Ok, we've encountered a non-hex character, cannot be a
            // valid line, skip and return 0's
            *A = 0;
            *D = 0;
            return;
        }

        Number <<= 4;
        Number += Digit;
        i++;
    }

    if (!Flag) {
        *A = 0;
        *D = 0;
    } else {
        *D = Number;
    }
}

void
HexFile::close()
{
    fclose(fp);
}
