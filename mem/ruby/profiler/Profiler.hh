/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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
 */

/*
   This file has been modified by Kevin Moore and Dan Nussbaum of the
   Scalable Systems Research Group at Sun Microsystems Laboratories
   (http://research.sun.com/scalable/) to support the Adaptive
   Transactional Memory Test Platform (ATMTP).

   Please send email to atmtp-interest@sun.com with feedback, questions, or
   to request future announcements about ATMTP.

   ----------------------------------------------------------------------

   File modification date: 2008-02-23

   ----------------------------------------------------------------------
*/

#ifndef __MEM_RUBY_PROFILER_PROFILER_HH__
#define __MEM_RUBY_PROFILER_PROFILER_HH__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/protocol/AccessType.hh"
#include "mem/ruby/protocol/PrefetchBit.hh"
#include "mem/ruby/protocol/RubyAccessMode.hh"
#include "mem/ruby/protocol/RubyRequestType.hh"
#include "params/RubySystem.hh"

namespace gem5
{

namespace ruby
{

class RubyRequest;
class AddressProfiler;

class Profiler
{
  public:
    Profiler(const RubySystemParams &params, RubySystem *rs);
    ~Profiler();

    RubySystem *m_ruby_system;

    void wakeup();
    void regStats();
    void collateStats();

    AddressProfiler* getAddressProfiler() { return m_address_profiler_ptr; }
    AddressProfiler* getInstructionProfiler() { return m_inst_profiler_ptr; }

    void addAddressTraceSample(const RubyRequest& msg, NodeID id);

    // added by SS
    bool getHotLines() const { return m_hot_lines; }
    bool getAllInstructions() const { return m_all_instructions; }

  private:
    // Private copy constructor and assignment operator
    Profiler(const Profiler& obj);
    Profiler& operator=(const Profiler& obj);

    AddressProfiler* m_address_profiler_ptr;
    AddressProfiler* m_inst_profiler_ptr;

    struct ProfilerStats : public statistics::Group
    {
        ProfilerStats(statistics::Group *parent, Profiler *profiler);

        struct PerRequestTypeStats : public statistics::Group
        {
            PerRequestTypeStats(statistics::Group *parent);

            // Histogram of the latency of each request type
            std::vector<statistics::Histogram *> m_typeLatencyHistSeqr;
            std::vector<statistics::Histogram *> m_typeLatencyHistCoalsr;

            // Histogram of the latency of requests that hit in the controller
            // connected to this sequencer for each type of request
            std::vector<statistics::Histogram *> m_hitTypeLatencyHistSeqr;

            // Histogram of the latency of requests that miss in the controller
            // connected to this sequencer for each type of request
            std::vector<statistics::Histogram *> m_missTypeLatencyHistSeqr;
            std::vector<statistics::Histogram *> m_missTypeLatencyHistCoalsr;
        } perRequestTypeStats;

        struct PerMachineTypeStats : public statistics::Group
        {
            PerMachineTypeStats(statistics::Group *parent);

            //! Histograms for profiling the latencies for requests that
            //! did not required external messages.
            std::vector<statistics::Histogram *> m_hitMachLatencyHistSeqr;

            //! Histograms for profiling the latencies for requests that
            //! required external messages.
            std::vector<statistics::Histogram *> m_missMachLatencyHistSeqr;
            std::vector<statistics::Histogram *> m_missMachLatencyHistCoalsr;

            //! Histograms for recording the breakdown of miss latency
            std::vector<statistics::Histogram *> m_IssueToInitialDelayHistSeqr;
            std::vector<statistics::Histogram *>
                m_InitialToForwardDelayHistSeqr;
            std::vector<statistics::Histogram *>
              m_ForwardToFirstResponseDelayHistSeqr;
            std::vector<statistics::Histogram *>
              m_FirstResponseToCompletionDelayHistSeqr;
            std::vector<statistics::Scalar *> m_IncompleteTimesSeqr;
            std::vector<statistics::Histogram *>
                m_IssueToInitialDelayHistCoalsr;
            std::vector<statistics::Histogram *>
                m_InitialToForwardDelayHistCoalsr;
            std::vector<statistics::Histogram *>
              m_ForwardToFirstResponseDelayHistCoalsr;
            std::vector<statistics::Histogram *>
              m_FirstResponseToCompletionDelayHistCoalsr;
        } perMachineTypeStats;

        struct PerRequestTypeMachineTypeStats : public statistics::Group
        {
            PerRequestTypeMachineTypeStats(statistics::Group *parent);

            //! Histograms for profiling the latencies for requests that
            //! did not required external messages.
            std::vector< std::vector<statistics::Histogram *> >
              m_hitTypeMachLatencyHistSeqr;

            //! Histograms for profiling the latencies for requests that
            //! required external messages.
            std::vector< std::vector<statistics::Histogram *> >
              m_missTypeMachLatencyHistSeqr;
            std::vector< std::vector<statistics::Histogram *> >
              m_missTypeMachLatencyHistCoalsr;
        } perRequestTypeMachineTypeStats;

        statistics::Histogram delayHistogram;
        std::vector<statistics::Histogram *> delayVCHistogram;

        //! Histogram for number of outstanding requests per cycle.
        statistics::Histogram m_outstandReqHistSeqr;
        statistics::Histogram m_outstandReqHistCoalsr;

        //! Histogram for holding latency profile of all requests.
        statistics::Histogram m_latencyHistSeqr;
        statistics::Histogram m_latencyHistCoalsr;

        //! Histogram for holding latency profile of all requests that
        //! hit in the controller connected to this sequencer.
        statistics::Histogram m_hitLatencyHistSeqr;

        //! Histogram for holding latency profile of all requests that
        //! miss in the controller connected to this sequencer.
        statistics::Histogram m_missLatencyHistSeqr;
        statistics::Histogram m_missLatencyHistCoalsr;
    };

    //added by SS
    const bool m_hot_lines;
    const bool m_all_instructions;
    const uint32_t m_num_vnets;


  public:
    ProfilerStats rubyProfilerStats;
};

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_PROFILER_PROFILER_HH__
