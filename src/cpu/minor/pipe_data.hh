/*
 * Copyright (c) 2013-2014 ARM Limited
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

/**
 * @file
 *
 *  Contains class definitions for data flowing between pipeline stages in
 *  the top-level structure portion of this model.  Latch types are also
 *  defined which pair forward/backward flowing data specific to each stage
 *  pair.
 *
 *  No post-configuration inter-stage communication should *ever* take place
 *  outside these classes (except for reservation!)
 */

#ifndef __CPU_MINOR_PIPE_DATA_HH__
#define __CPU_MINOR_PIPE_DATA_HH__

#include "cpu/minor/buffers.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/base.hh"

namespace Minor
{

/** Forward data betwen Execute and Fetch1 carrying change-of-address/stream
 *  information. */
class BranchData /* : public ReportIF, public BubbleIF */
{
  public:
    enum Reason
    {
        /* *** No change of stream (information to branch prediction) */

        /* Don't branch at all (bubble) */
        NoBranch,
        /* Don't branch, but here's the details of a correct prediction
         * that was executed */
        CorrectlyPredictedBranch,

        /* *** Change of stream */

        /* Take an unpredicted branch */
        UnpredictedBranch,
        /* Take a branch on branch prediction data (from Fetch2) */
        BranchPrediction,
        /* Prediction of wrong target PC */
        BadlyPredictedBranchTarget,
        /* Bad branch prediction (didn't actually branch).  Need to branch
         *  back to correct stream.  If the target is wrong, use
         *  BadlyPredictedBranchTarget */
        BadlyPredictedBranch,
        /* Suspend fetching for this thread (inst->id.threadId).
         * This will be woken up by another stream changing branch so
         * count it as stream changing itself and expect pc to be the PC
         * of the next instruction */
        SuspendThread,
        /* Branch from an interrupt (no instruction) */
        Interrupt,
        /* Stop fetching in anticipation of of draining */
        HaltFetch
    };

    /** Is a request with this reason actually a request to change the
     *  PC rather than a bubble or branch prediction information */
    static bool isStreamChange(const BranchData::Reason reason);

    /** Is a request with this reason actually a 'real' branch, that is,
     *  a stream change that's not just an instruction to Fetch1 to halt
     *  or wake up */
    static bool isBranch(const BranchData::Reason reason);

  public:
    /** Explanation for this branch */
    Reason reason;

    /** ThreadID associated with branch */
    ThreadID threadId;

    /** Sequence number of new stream/prediction to be adopted */
    InstSeqNum newStreamSeqNum;
    InstSeqNum newPredictionSeqNum;

    /** Starting PC of that stream */
    TheISA::PCState target;

    /** Instruction which caused this branch */
    MinorDynInstPtr inst;

  public:
    BranchData() :
        reason(NoBranch), threadId(InvalidThreadID), newStreamSeqNum(0),
        newPredictionSeqNum(0), target(TheISA::PCState(0)),
        inst(MinorDynInst::bubble())
    { }

    BranchData(
        Reason reason_,
        ThreadID thread_id,
        InstSeqNum new_stream_seq_num,
        InstSeqNum new_prediction_seq_num,
        TheISA::PCState target,
        MinorDynInstPtr inst_) :
        reason(reason_),
        threadId(thread_id),
        newStreamSeqNum(new_stream_seq_num),
        newPredictionSeqNum(new_prediction_seq_num),
        target(target),
        inst(inst_)
    { }

    /** BubbleIF interface */
    static BranchData bubble() { return BranchData(); }
    bool isBubble() const { return reason == NoBranch; }

    /** As static isStreamChange but on this branch data */
    bool isStreamChange() const { return isStreamChange(reason); }

    /** As static isBranch but on this branch data */
    bool isBranch() const { return isBranch(reason); }

    /** ReportIF interface */
    void reportData(std::ostream &os) const;
};

/** Print a branch reason enum */
std::ostream &operator <<(std::ostream &os, BranchData::Reason reason);

/** Print BranchData contents in a format suitable for DPRINTF comments, not
 *  for MinorTrace */
std::ostream &operator <<(std::ostream &os, const BranchData &branch);

/** Line fetch data in the forward direction.  Contains a single cache line
 *  (or fragment of a line), its address, a sequence number assigned when
 *  that line was fetched and a bubbleFlag that can allow ForwardLineData to
 *  be used to represent the absence of line data in a pipeline. */
class ForwardLineData /* : public ReportIF, public BubbleIF */
{
  private:
    /** This line is a bubble.  No other data member is required to be valid
     *  if this is true */
    bool bubbleFlag;

  public:
    /** First byte address in the line.  This is allowed to be
     *  <= pc.instAddr() */
    Addr lineBaseAddr;

    /** PC of the first requested inst within this line */
    TheISA::PCState pc;

    /** Explicit line width, don't rely on data.size */
    unsigned int lineWidth;

  public:
    /** This line has a fault.  The bubble flag will be false and seqNums
     *  will be valid but no data will */
    Fault fault;

    /** Thread, stream, prediction ... id of this line */
    InstId id;

    /** Line data.  line[0] is the byte at address pc.instAddr().  Data is
     *  only valid upto lineWidth - 1. */
    uint8_t *line;

    /** Packet from which the line is taken */
    Packet *packet;

  public:
    ForwardLineData() :
        bubbleFlag(true),
        lineBaseAddr(0),
        lineWidth(0),
        fault(NoFault),
        line(NULL),
        packet(NULL)
    {
        /* Make lines bubbles by default */
    }

    ~ForwardLineData() { line = NULL; }

  public:
    /** This is a fault, not a line */
    bool isFault() const { return fault != NoFault; }

    /** Set fault and possible clear the bubble flag */
    void setFault(Fault fault_);

    /** In-place initialise a ForwardLineData, freeing and overridding the
     *  line */
    void allocateLine(unsigned int width_);

    /** Use the data from a packet as line instead of allocating new
     *  space.  On destruction of this object, the packet will be destroyed */
    void adoptPacketData(Packet *packet);

    /** Free this ForwardLineData line.  Note that these are shared between
     *  line objects and so you must be careful when deallocating them.
     *  Copying of ForwardLineData can, therefore, be done by default copy
     *  constructors/assignment */
    void freeLine();

    /** BubbleIF interface */
    static ForwardLineData bubble() { return ForwardLineData(); }
    bool isBubble() const { return bubbleFlag; }

    /** ReportIF interface */
    void reportData(std::ostream &os) const;
};

/** Maximum number of instructions that can be carried by the pipeline. */
const unsigned int MAX_FORWARD_INSTS = 16;

/** Forward flowing data between Fetch2,Decode,Execute carrying a packet of
 *  instructions of a width appropriate to the configured stage widths.
 *  Also carries exception information where instructions are not valid */
class ForwardInstData /* : public ReportIF, public BubbleIF */
{
  public:
    /** Array of carried insts, ref counted */
    MinorDynInstPtr insts[MAX_FORWARD_INSTS];

    /** The number of insts slots that can be expected to be valid insts */
    unsigned int numInsts;

    /** Thread associated with these instructions */
    ThreadID threadId;

  public:
    explicit ForwardInstData(unsigned int width = 0,
                             ThreadID tid = InvalidThreadID);

    ForwardInstData(const ForwardInstData &src);

  public:
    /** Number of instructions carried by this object */
    unsigned int width() const { return numInsts; }

    /** Copy the inst array only as far as numInsts */
    ForwardInstData &operator =(const ForwardInstData &src);

    /** Resize a bubble/empty ForwardInstData and fill with bubbles */
    void resize(unsigned int width);

    /** Fill with bubbles from 0 to width() - 1 */
    void bubbleFill();

    /** BubbleIF interface */
    bool isBubble() const;

    /** ReportIF interface */
    void reportData(std::ostream &os) const;
};

}

#endif /* __CPU_MINOR_PIPE_DATA_HH__ */
