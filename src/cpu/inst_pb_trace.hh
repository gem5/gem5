/*
 * Copyright (c) 2014 ARM Limited
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
 * Authors: Ali Saidi
 */

#ifndef __CPU_INST_PB_TRACE_HH__
#define __CPU_INST_PB_TRACE_HH__

#include "arch/types.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst_fwd.hh"
#include "params/InstPBTrace.hh"
#include "proto/protoio.hh"
#include "sim/insttracer.hh"

class ThreadContext;

namespace ProtoMessage {
class Inst;
}

namespace Trace {

/**
 * This in an instruction tracer that records the flow of instructions through
 * multiple cpus and systems to a protobuf file specified by proto/inst.proto
 * for further analysis.
 */

class InstPBTraceRecord : public InstRecord
{
  public:
    InstPBTraceRecord(InstPBTrace& _tracer, Tick when, ThreadContext *tc,
                      const StaticInstPtr si, TheISA::PCState pc,
                      const StaticInstPtr mi = NULL)
        : InstRecord(when, tc, si, pc, mi), tracer(_tracer)
    {}

    /** called by the cpu when the instruction commits.
     * This implementation of dump calls InstPBTrace to output the contents to a
     * protobuf file
     */
    void dump() override;

  protected:
    InstPBTrace& tracer;

};

class InstPBTrace : public InstTracer
{
 public:
    InstPBTrace(const InstPBTraceParams *p);
    virtual ~InstPBTrace();

    InstPBTraceRecord* getInstRecord(Tick when, ThreadContext *tc, const
                                    StaticInstPtr si, TheISA::PCState pc, const
                                    StaticInstPtr mi = NULL) override;

  protected:
    std::unique_ptr<uint8_t []> buf;
    size_t bufSize;

    /** One output stream for the entire simulation.
     * We encode the CPU & system ID so all we need is a single file
     */
    static ProtoOutputStream *traceStream;


    /** This is the message were working on writing. The majority of the message
     * exists however the memory accesses will be delayed.
     */
    ProtoMessage::Inst *curMsg;

    /** Create the output file and write the header into it
     * @param filename the file to create (if ends with .gz it will be
     * compressed)
     */
    void createTraceFile(std::string filename);

    /** If there is a pending message still write it out and then close the file
     */
    void closeStreams();

    /** Write an instruction to the trace file
     * @param tc thread context for the cpu ID
     * @param si for the machInst and opClass
     * @param pc for the PC Addr
     */
    void traceInst(ThreadContext *tc, StaticInstPtr si, TheISA::PCState pc);

    /** Write a memory request to the trace file as part of the cur instruction
     * @param si for the machInst and opClass
     * @param a address of the request
     * @param s size of the request
     * @param f flags for the request
     */
    void traceMem(StaticInstPtr si, Addr a, Addr s, unsigned f);

    friend class InstPBTraceRecord;
};
} // namespace Trace
#endif // __CPU_INST_PB_TRACE_HH__
