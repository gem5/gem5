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
 */

#include "cpu/inst_pb_trace.hh"

#include "base/callback.hh"
#include "base/output.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecEnable.hh"
#include "params/InstPBTrace.hh"
#include "proto/inst.pb.h"
#include "sim/core.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

namespace trace {

ProtoOutputStream *InstPBTrace::traceStream;

void
InstPBTraceRecord::dump()
{
    // We're trying to build an instruction trace so we just want macro-ops and
    // instructions that aren't macro-oped
    if ((macroStaticInst && staticInst->isFirstMicroop()) ||
            !staticInst->isMicroop()) {
        tracer.traceInst(thread, staticInst, *pc);
    }

    // If this instruction accessed memory lets record it
    if (getMemValid())
        tracer.traceMem(staticInst, getAddr(), getSize(), getFlags());
}

InstPBTrace::InstPBTrace(const InstPBTraceParams &p)
    : InstTracer(p), buf(nullptr), bufSize(0), curMsg(nullptr)
{
    // Create our output file
    createTraceFile(p.file_name);
}

void
InstPBTrace::createTraceFile(std::string filename)
{
    // Since there is only one output file for all tracers check if it exists
    if (traceStream)
        return;

    traceStream = new ProtoOutputStream(simout.resolve(filename));

    // Output the header
    ProtoMessage::InstHeader header_msg;
    header_msg.set_obj_id("gem5 generated instruction trace");
    header_msg.set_ver(0);
    header_msg.set_tick_freq(sim_clock::Frequency);
    header_msg.set_has_mem(true);
    traceStream->write(header_msg);

    // get a callback when we exit so we can close the file
    registerExitCallback([this]() { closeStreams(); });
}

void
InstPBTrace::closeStreams()
{
    if (curMsg) {
        traceStream->write(*curMsg);
        delete curMsg;
        curMsg = NULL;
    }

    if (!traceStream)
        return;

    delete traceStream;
    traceStream = NULL;
}

InstPBTrace::~InstPBTrace()
{
    closeStreams();
}

InstPBTraceRecord*
InstPBTrace::getInstRecord(Tick when, ThreadContext *tc, const StaticInstPtr si,
                           const PCStateBase &pc, const StaticInstPtr mi)
{
    // Only record the trace if Exec debugging is enabled
    if (!debug::ExecEnable)
        return NULL;

    return new InstPBTraceRecord(*this, when, tc, si, pc, mi);

}

void
InstPBTrace::traceInst(ThreadContext *tc, StaticInstPtr si,
        const PCStateBase &pc)
{
    if (curMsg) {
        //TODO if we are running multi-threaded I assume we'd need a lock here
        traceStream->write(*curMsg);
        delete curMsg;
        curMsg = NULL;
    }

    size_t instSize = si->asBytes(buf.get(), bufSize);
    if (instSize > bufSize) {
        bufSize = instSize;
        buf.reset(new uint8_t[bufSize]);
        instSize = si->asBytes(buf.get(), bufSize);
    }

    // Create a new instruction message and fill out the fields
    curMsg = new ProtoMessage::Inst;
    curMsg->set_pc(pc.instAddr());
    if (instSize == sizeof(uint32_t)) {
        curMsg->set_inst(letoh(*reinterpret_cast<uint32_t *>(buf.get())));
    } else if (instSize) {
        curMsg->set_inst_bytes(
            std::string(reinterpret_cast<const char *>(buf.get()), bufSize));
    }
    curMsg->set_cpuid(tc->cpuId());
    curMsg->set_tick(curTick());
    curMsg->set_type(static_cast<ProtoMessage::Inst_InstType>(si->opClass()));
}

void
InstPBTrace::traceMem(StaticInstPtr si, Addr a, Addr s, unsigned f)
{
    panic_if(!curMsg, "Memory access w/o msg?!");

    // We do a poor job identifying macro-ops that are load/stores
    curMsg->set_type(static_cast<ProtoMessage::Inst_InstType>(si->opClass()));

    ProtoMessage::Inst::MemAccess *mem_msg = curMsg->add_mem_access();
    mem_msg->set_addr(a);
    mem_msg->set_size(s);
    mem_msg->set_mem_flags(f);

}

} // namespace trace
} // namespace gem5
