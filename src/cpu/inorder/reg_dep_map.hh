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
 * Authors: Korey Sewell
 *
 */

#ifndef CPU_INORDER_REG_DEP_MAP_HH
#define CPU_INORDER_REG_DEP_MAP_HH

#include <list>
#include <vector>

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/pipeline_traits.hh"

class InOrderCPU;

class RegDepMap
{
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    RegDepMap(int size = TheISA::TotalNumRegs);

    ~RegDepMap();

    std::string name();

    void setCPU(InOrderCPU *_cpu);

    /** Clear the Entire Map */
    void clear();

    /** Insert all of a instruction's destination registers into map*/
    void insert(DynInstPtr inst);

    /** Insert an instruction into a specific destination register index
     *  onto map
     */
    void insert(unsigned idx, DynInstPtr inst);

    /** Remove all of a instruction's destination registers into map*/
    void remove(DynInstPtr inst);

    /** Remove a specific instruction and dest. register index from map*/
    void remove(unsigned idx, DynInstPtr inst);

    /** Remove Front instruction from a destination register */
    void removeFront(unsigned idx, DynInstPtr inst);

    /** Is the current instruction able to read from this
     *  destination register?
     */
    bool canRead(unsigned idx, DynInstPtr inst);

    /** Is the current instruction able to get a forwarded value from
     *  another instruction for this destination register?
     */
    DynInstPtr canForward(unsigned reg_idx, DynInstPtr inst);

    /** find an instruction to forward/bypass a value from */
    DynInstPtr findBypassInst(unsigned idx);

    /** Is the current instruction able to write to this
     *  destination register?
     */
    bool canWrite(unsigned idx, DynInstPtr inst);

    /** Size of Dependency of Map */
    int depSize(unsigned idx);

    void dump();
    
  protected:
    // Eventually make this a map of lists for
    // efficiency sake!
    std::vector<std::list<DynInstPtr> > regMap;

    InOrderCPU *cpu;
};

#endif







