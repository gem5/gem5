/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
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
 *
 * Authors: Stan Czerniawski
 */

#ifndef __DEV_ARM_SMMU_V3_PTOPS_HH__
#define __DEV_ARM_SMMU_V3_PTOPS_HH__

#include <stdint.h>

#include "base/types.hh"

struct PageTableOps
{
    typedef int64_t pte_t;

    virtual bool isValid(pte_t pte, unsigned level) const = 0;
    virtual bool isLeaf(pte_t pte, unsigned level) const = 0;
    virtual bool isWritable(pte_t pte, unsigned level, bool stage2) const = 0;
    virtual Addr nextLevelPointer(pte_t pte, unsigned level) const = 0;
    virtual Addr index(Addr va, unsigned level) const = 0;
    virtual Addr pageMask(pte_t pte, unsigned level) const = 0;
    virtual Addr walkMask(unsigned level) const = 0;
    virtual unsigned firstLevel(uint8_t tsz) const = 0;
    virtual unsigned lastLevel() const = 0;
};

struct V7LPageTableOps : public PageTableOps
{
    bool isValid(pte_t pte, unsigned level) const override;
    bool isLeaf(pte_t pte, unsigned level) const override;
    bool isWritable(pte_t pte, unsigned level, bool stage2) const override;
    Addr nextLevelPointer(pte_t pte, unsigned level) const override;
    Addr index(Addr va, unsigned level) const override;
    Addr pageMask(pte_t pte, unsigned level) const override;
    Addr walkMask(unsigned level) const override;
    unsigned firstLevel(uint8_t tsz) const override;
    unsigned lastLevel() const override;
};

struct V8PageTableOps4k : public PageTableOps
{
    bool isValid(pte_t pte, unsigned level) const override;
    bool isLeaf(pte_t pte, unsigned level) const override;
    bool isWritable(pte_t pte, unsigned level, bool stage2) const override;
    Addr nextLevelPointer(pte_t pte, unsigned level) const override;
    Addr index(Addr va, unsigned level) const override;
    Addr pageMask(pte_t pte, unsigned level) const override;
    Addr walkMask(unsigned level) const override;
    unsigned firstLevel(uint8_t tsz) const override;
    unsigned lastLevel() const override;
};

struct V8PageTableOps16k : public PageTableOps
{
    bool isValid(pte_t pte, unsigned level) const override;
    bool isLeaf(pte_t pte, unsigned level) const override;
    bool isWritable(pte_t pte, unsigned level, bool stage2) const override;
    Addr nextLevelPointer(pte_t pte, unsigned level) const override;
    Addr index(Addr va, unsigned level) const override;
    Addr pageMask(pte_t pte, unsigned level) const override;
    Addr walkMask(unsigned level) const override;
    unsigned firstLevel(uint8_t tsz) const override;
    unsigned lastLevel() const override;
};

struct V8PageTableOps64k : public PageTableOps
{
    bool isValid(pte_t pte, unsigned level) const override;
    bool isLeaf(pte_t pte, unsigned level) const override;
    bool isWritable(pte_t pte, unsigned level, bool stage2) const override;
    Addr nextLevelPointer(pte_t pte, unsigned level) const override;
    Addr index(Addr va, unsigned level) const override;
    Addr pageMask(pte_t pte, unsigned level) const override;
    Addr walkMask(unsigned level) const override;
    unsigned firstLevel(uint8_t tsz) const override;
    unsigned lastLevel() const override;
};

#endif /* __DEV_ARM_SMMU_V3_PTOPS_HH__ */
