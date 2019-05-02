/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

#include "arch/alpha/osfpal.hh"

const char *
PAL::name(int index)
{
    static const char *strings[PAL::NumCodes] = {
        // Priviledged PAL instructions
        "halt",         // 0x00
        "cflush",       // 0x01
        "draina",       // 0x02
        0,              // 0x03
        0,              // 0x04
        0,              // 0x05
        0,              // 0x06
        0,              // 0x07
        0,              // 0x08
        "cserve",       // 0x09
        "swppal",       // 0x0a
        0,              // 0x0b
        0,              // 0x0c
        "wripir",       // 0x0d
        0,              // 0x0e
        0,              // 0x0f
        "rdmces",       // 0x10
        "wrmces",       // 0x11
        0,              // 0x12
        0,              // 0x13
        0,              // 0x14
        0,              // 0x15
        0,              // 0x16
        0,              // 0x17
        0,              // 0x18
        0,              // 0x19
        0,              // 0x1a
        0,              // 0x1b
        0,              // 0x1c
        0,              // 0x1d
        0,              // 0x1e
        0,              // 0x1f
        0,              // 0x20
        0,              // 0x21
        0,              // 0x22
        0,              // 0x23
        0,              // 0x24
        0,              // 0x25
        0,              // 0x26
        0,              // 0x27
        0,              // 0x28
        0,              // 0x29
        0,              // 0x2a
        "wrfen",        // 0x2b
        0,              // 0x2c
        "wrvptptr",     // 0x2d
        0,              // 0x2e
        0,              // 0x2f
        "swpctx",       // 0x30
        "wrval",        // 0x31
        "rdval",        // 0x32
        "tbi",          // 0x33
        "wrent",        // 0x34
        "swpipl",       // 0x35
        "rdps",         // 0x36
        "wrkgp",        // 0x37
        "wrusp",        // 0x38
        "wrperfmon",    // 0x39
        "rdusp",        // 0x3a
        0,              // 0x3b
        "whami",        // 0x3c
        "retsys",       // 0x3d
        "wtint",        // 0x3e
        "rti",          // 0x3f
        0,              // 0x40
        0,              // 0x41
        0,              // 0x42
        0,              // 0x43
        0,              // 0x44
        0,              // 0x45
        0,              // 0x46
        0,              // 0x47
        0,              // 0x48
        0,              // 0x49
        0,              // 0x4a
        0,              // 0x4b
        0,              // 0x4c
        0,              // 0x4d
        0,              // 0x4e
        0,              // 0x4f
        0,              // 0x50
        0,              // 0x51
        0,              // 0x52
        0,              // 0x53
        0,              // 0x54
        0,              // 0x55
        0,              // 0x56
        0,              // 0x57
        0,              // 0x58
        0,              // 0x59
        0,              // 0x5a
        0,              // 0x5b
        0,              // 0x5c
        0,              // 0x5d
        0,              // 0x5e
        0,              // 0x5f
        0,              // 0x60
        0,              // 0x61
        0,              // 0x62
        0,              // 0x63
        0,              // 0x64
        0,              // 0x65
        0,              // 0x66
        0,              // 0x67
        0,              // 0x68
        0,              // 0x69
        0,              // 0x6a
        0,              // 0x6b
        0,              // 0x6c
        0,              // 0x6d
        0,              // 0x6e
        0,              // 0x6f
        0,              // 0x70
        0,              // 0x71
        0,              // 0x72
        0,              // 0x73
        0,              // 0x74
        0,              // 0x75
        0,              // 0x76
        0,              // 0x77
        0,              // 0x78
        0,              // 0x79
        0,              // 0x7a
        0,              // 0x7b
        0,              // 0x7c
        0,              // 0x7d
        0,              // 0x7e
        0,              // 0x7f

        // Unpriviledged PAL instructions
        "bpt",          // 0x80
        "bugchk",       // 0x81
        0,              // 0x82
        "callsys",      // 0x83
        0,              // 0x84
        0,              // 0x85
        "imb",          // 0x86
        0,              // 0x87
        0,              // 0x88
        0,              // 0x89
        0,              // 0x8a
        0,              // 0x8b
        0,              // 0x8c
        0,              // 0x8d
        0,              // 0x8e
        0,              // 0x8f
        0,              // 0x90
        0,              // 0x91
        "urti",         // 0x92
        0,              // 0x93
        0,              // 0x94
        0,              // 0x95
        0,              // 0x96
        0,              // 0x97
        0,              // 0x98
        0,              // 0x99
        0,              // 0x9a
        0,              // 0x9b
        0,              // 0x9c
        0,              // 0x9d
        "rdunique",     // 0x9e
        "wrunique",     // 0x9f
        0,              // 0xa0
        0,              // 0xa1
        0,              // 0xa2
        0,              // 0xa3
        0,              // 0xa4
        0,              // 0xa5
        0,              // 0xa6
        0,              // 0xa7
        0,              // 0xa8
        0,              // 0xa9
        "gentrap",      // 0xaa
        0,              // 0xab
        0,              // 0xac
        0,              // 0xad
        "clrfen",       // 0xae
        0,              // 0xaf
        0,              // 0xb0
        0,              // 0xb1
        0,              // 0xb2
        0,              // 0xb3
        0,              // 0xb4
        0,              // 0xb5
        0,              // 0xb6
        0,              // 0xb7
        0,              // 0xb8
        0,              // 0xb9
        0,              // 0xba
        0,              // 0xbb
        0,              // 0xbc
        0,              // 0xbd
        "nphalt",       // 0xbe
        "copypal",      // 0xbf
    };

    if (index > NumCodes || index < 0)
        return 0;

    return strings[index];
}
