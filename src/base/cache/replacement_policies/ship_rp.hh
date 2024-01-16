/**
 * Copyright (c) 2019, 2020 Inria
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

/**
 * @file
 * Declaration of a the SHiP Replacement Policy, as described in "SHiP:
 * Signature-based Hit Predictor for High Performance Caching", by
 * Wu et al.
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__

#include <cstddef>
#include <vector>

#include "base/cache/replacement_policies/brrip_rp.hh"
#include "base/compiler.hh"
#include "base/sat_counter.hh"
#include "mem/packet.hh"

namespace gem5
{

struct SHiPRPParams;
struct SHiPMemRPParams;
struct SHiPPCRPParams;

namespace replacement_policy
{

class SHiP : public BRRIP
{
  protected:
    typedef std::size_t SignatureType;

    /** SHiP-specific implementation of replacement data. */
    class SHiPReplData : public BRRIPReplData
    {
      private:
        /** Signature that caused the insertion of this entry. */
        SignatureType signature;

        /** Outcome of insertion; set to one if entry is re-referenced. */
        bool outcome;

      public:
        SHiPReplData(int num_bits);

        /** Get entry's signature. */
        SignatureType getSignature() const;

        /**
         * Set this entry's signature and reset outcome.
         *
         * @param signature New signature value/
         */
        void setSignature(SignatureType signature);

        /** Set that this entry has been re-referenced. */
        void setReReferenced();

        /**
         * Get whether entry has been re-referenced since insertion.
         *
         * @return True if entry has been re-referenced since insertion.
         */
        bool wasReReferenced() const;
    };

    /**
     * Saturation percentage at which an entry starts being inserted as
     * intermediate re-reference.
     */
    const double insertionThreshold;

    /**
     * Signature History Counter Table; learns the re-reference behavior
     * of a signature. A zero entry provides a strong indication that
     * future lines brought by that signature will not receive any hits.
     */
    std::vector<SatCounter8> SHCT;

    /**
     * Extract signature from packet.
     *
     * @param pkt The packet to extract a signature from.
     * @return The signature extracted.
     */
    virtual SignatureType getSignature(const PacketPtr pkt) const = 0;

  public:
    typedef SHiPRPParams Params;
    SHiP(const Params &p);
    ~SHiP() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Updates predictor and invalidate data.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Updates predictor and assigns RRPV values of Table 3.
     *
     * @param replacement_data Replacement data to be touched.
     * @param pkt Packet that generated this hit.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Updates predictor and assigns RRPV values of Table 3.
     *
     * @param replacement_data Replacement data to be reset.
     * @param pkt Packet that generated this miss.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data,
        const PacketPtr pkt) override;
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
        override;

    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;
};

/** SHiP that Uses memory addresses as signatures. */
class SHiPMem : public SHiP
{
  protected:
    SignatureType getSignature(const PacketPtr pkt) const override;

  public:
    SHiPMem(const SHiPMemRPParams &p);
    ~SHiPMem() = default;
};

/** SHiP that Uses PCs as signatures. */
class SHiPPC : public SHiP
{
  private:
    /** Signature to be used when no PC is provided in an access. */
    const SignatureType NO_PC_SIGNATURE = 0;

  protected:
    SignatureType getSignature(const PacketPtr pkt) const override;

  public:
    SHiPPC(const SHiPPCRPParams &p);
    ~SHiPPC() = default;
};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SHIP_RP_HH__
