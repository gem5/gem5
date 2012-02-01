/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 *          Steve Reinhardt
 *          Nathan Binkert
 */

/**
 * @file
 * Declaration of a base replacement policy class.
 */

#ifndef __REPL_HH__
#define __REPL_HH__

#include <list>
#include <string>

#include "base/types.hh"
#include "cpu/smt.hh"
#include "sim/sim_object.hh"

class IIC;

/**
 * A pure virtual base class that defines the interface of a replacement
 * policy.
 */
class Repl : public SimObject
{
 public:
    /** Pointer to the IIC using this policy. */
    IIC *iic;

    Repl (const Params *params)
        : SimObject(params)
    {
        iic = NULL;
    }

    /**
     * Set the back pointer to the IIC.
     * @param iic_ptr Pointer to the IIC.
     */
    void setIIC(IIC *iic_ptr)
    {
        iic = iic_ptr;
    }

    /**
     * Returns the tag pointer of the cache block to replace.
     * @return The tag to replace.
     */
    virtual unsigned long getRepl() = 0;

    /**
     * Return an array of N tag pointers to replace.
     * @param n The number of tag pointer to return.
     * @return An array of tag pointers to replace.
     */
    virtual unsigned long  *getNRepl(int n) = 0;

    /**
     * Update replacement data
     */
    virtual void doAdvance(std::list<unsigned long> &demoted) = 0;

     /**
     * Add a tag to the replacement policy and return a pointer to the
     * replacement entry.
     * @param tag_index The tag to add.
     * @return The replacement entry.
     */
    virtual void* add(unsigned long tag_index) = 0;

    /**
     * Register statistics.
     * @param name The name to prepend to statistic descriptions.
     */
    virtual void regStatsWithSuffix(const std::string name) = 0;

    /**
     * Update the tag pointer to when the tag moves.
     * @param re The replacement entry of the tag.
     * @param old_index The old tag pointer.
     * @param new_index The new tag pointer.
     * @return 1 if successful, 0 otherwise.
     */
    virtual int fixTag(void *re, unsigned long old_index,
                       unsigned long new_index) = 0;

    /**
     * Remove this entry from the replacement policy.
     * @param re The replacement entry to remove
     */
    virtual void removeEntry(void *re) = 0;
};

#endif /* SMT_REPL_HH */
