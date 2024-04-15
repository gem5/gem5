/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
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

#ifndef __ARCH_ARM_LINUX_ATAG_HH__
#define __ARCH_ARM_LINUX_ATAG_HH__

#include <cstring>
#include <string>

#include "base/types.hh"

namespace gem5
{

enum
{
    CoreTag = 0x54410001,
    MemTag = 0x54410002,
    RevTag = 0x54410007,
    SerialTag = 0x54410006,
    CmdTag = 0x54410009,
    NoneTag = 0x00000000
};

class AtagHeader
{
  protected:
    uint32_t *storage;
    uint32_t _size;

  public:
    /** Tag (normally starts with 'T''A' and 16 bits of number */
    virtual uint32_t tag() = 0;

    /** If the header should be 0 size */
    virtual bool
    null()
    {
        return false;
    }

    uint32_t
    size() const
    {
        return _size;
    }

    AtagHeader(uint32_t s) : _size(s) { storage = new uint32_t[size()]; }

    virtual ~AtagHeader() { delete[] storage; }

    uint32_t
    copyOut(uint8_t *p)
    {
        storage[0] = null() ? 0 : size();
        storage[1] = tag();
        memcpy(p, storage, size() << 2);
        return size() << 2;
    }
};

class AtagCore : public AtagHeader
{
  public:
    static const uint32_t Size = 5;

    uint32_t
    tag()
    {
        return CoreTag;
    }

    void
    flags(uint32_t i)
    {
        storage[2] = i;
    }

    void
    pagesize(uint32_t i)
    {
        storage[3] = i;
    }

    void
    rootdev(uint32_t i)
    {
        storage[4] = i;
    }

    AtagCore() : AtagHeader(Size) {}
};

class AtagMem : public AtagHeader
{
  public:
    static const uint32_t Size = 4;

    uint32_t
    tag()
    {
        return MemTag;
    }

    void
    memSize(uint32_t i)
    {
        storage[2] = i;
    }

    void
    memStart(uint32_t i)
    {
        storage[3] = i;
    }

    AtagMem() : AtagHeader(Size) {}
};

class AtagRev : public AtagHeader
{
  public:
    static const uint32_t Size = 3;

    uint32_t
    tag()
    {
        return RevTag;
    }

    void
    rev(uint32_t i)
    {
        storage[2] = i;
    }

    AtagRev() : AtagHeader(Size) {}
};

class AtagSerial : public AtagHeader
{
  public:
    static const uint32_t Size = 4;

    uint32_t
    tag()
    {
        return SerialTag;
    }

    void
    sn(uint64_t i)
    {
        storage[2] = (uint32_t)i;
        storage[3] = i >> 32;
    }

    AtagSerial() : AtagHeader(Size) {}
};

class AtagCmdline : public AtagHeader
{
  public:
    static const uint32_t Size = 3;

    uint32_t
    tag()
    {
        return CmdTag;
    }

    void
    cmdline(const std::string &s)
    {
        // Add one for null terminator
        int len = s.length() + 1;

        // 2 + ceiling(len/4)
        _size = 2 + ((len + 3) >> 2);

        delete[] storage;
        storage = new uint32_t[size()];
        // Initialize the last byte of memory here beacuse it might be slightly
        // longer than needed and mis-speculation of the NULL in the O3 CPU can
        // change stats ever so slightly when that happens.
        storage[size() - 1] = 0;
        strcpy((char *)&storage[2], s.c_str());
    }

    AtagCmdline() : AtagHeader(Size) {}
};

class AtagNone : public AtagHeader
{
  public:
    static const uint32_t Size = 2;

    virtual bool
    null()
    {
        return true;
    }

    uint32_t
    tag()
    {
        return NoneTag;
    }

    AtagNone() : AtagHeader(Size) {}
};

/*
//
// example ARM Linux bootloader code
// this example is distributed under the BSD licence
// Code taken from
http://www.simtec.co.uk/products/SWLINUX/files/booting_article.html
///

// list of possible tags
#define ATAG_NONE       0x00000000
#define ATAG_CORE       0x54410001
#define ATAG_MEM        0x54410002
#define ATAG_VIDEOTEXT  0x54410003
#define ATAG_RAMDISK    0x54410004
#define ATAG_INITRD2    0x54420005
#define ATAG_SERIAL     0x54410006
#define ATAG_REVISION   0x54410007
#define ATAG_VIDEOLFB   0x54410008
#define ATAG_CMDLINE    0x54410009

// structures for each atag
struct atag_header
{
        u32 size; // length of tag in words including this header
        u32 tag;  // tag type
};

struct atag_core
{
        u32 flags;
        u32 pagesize;
        u32 rootdev;
};

struct atag_mem
{
        u32     size;
        u32     start;
};

struct atag_videotext
{
        u8              x;
        u8              y;
        u16             video_page;
        u8              video_mode;
        u8              video_cols;
        u16             video_ega_bx;
        u8              video_lines;
        u8              video_isvga;
        u16             video_points;
};

struct atag_ramdisk
{
        u32 flags;
        u32 size;
        u32 start;
};

struct atag_initrd2
{
        u32 start;
        u32 size;
};

struct atag_serialnr
{
        u32 low;
        u32 high;
};

struct atag_revision
{
        u32 rev;
};

struct atag_videolfb
{
        u16             lfb_width;
        u16             lfb_height;
        u16             lfb_depth;
        u16             lfb_linelength;
        u32             lfb_base;
        u32             lfb_size;
        u8              red_size;
        u8              red_pos;
        u8              green_size;
        u8              green_pos;
        u8              blue_size;
        u8              blue_pos;
        u8              rsvd_size;
        u8              rsvd_pos;
};

struct atag_cmdline
{
        char    cmdline[1];
};

struct atag
{
        struct atag_header hdr;
        union
        {
                struct atag_core         core;
                struct atag_mem          mem;
                struct atag_videotext    videotext;
                struct atag_ramdisk      ramdisk;
                struct atag_initrd2      initrd2;
                struct atag_serialnr     serialnr;
                struct atag_revision     revision;
                struct atag_videolfb     videolfb;
                struct atag_cmdline      cmdline;
        } u;
};
*/

} // namespace gem5

#endif // __ARCH_ARM_LINUX_ATAG_HH__
