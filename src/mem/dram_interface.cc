/*
 * Copyright (c) 2010-2020 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
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

#include "mem/dram_interface.hh"

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/trace.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMPower.hh"
#include "debug/DRAMState.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace Data;

namespace memory
{

std::pair<MemPacketQueue::iterator, Tick>
DRAMInterface::chooseNextFRFCFS(MemPacketQueue& queue, Tick min_col_at) const
{
    std::vector<uint32_t> earliest_banks(ranksPerChannel, 0);

    // Has minBankPrep been called to populate earliest_banks?
    bool filled_earliest_banks = false;
    // can the PRE/ACT sequence be done without impacting utlization?
    bool hidden_bank_prep = false;

    // search for seamless row hits first, if no seamless row hit is
    // found then determine if there are other packets that can be issued
    // without incurring additional bus delay due to bank timing
    // Will select closed rows first to enable more open row possibilies
    // in future selections
    bool found_hidden_bank = false;

    // remember if we found a row hit, not seamless, but bank prepped
    // and ready
    bool found_prepped_pkt = false;

    // if we have no row hit, prepped or not, and no seamless packet,
    // just go for the earliest possible
    bool found_earliest_pkt = false;

    Tick selected_col_at = MaxTick;
    auto selected_pkt_it = queue.end();

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        MemPacket* pkt = *i;

        // select optimal DRAM packet in Q
        if (pkt->isDram() && (pkt->pseudoChannel == pseudoChannel)) {
            const Bank& bank = ranks[pkt->rank]->banks[pkt->bank];
            const Tick col_allowed_at = pkt->isRead() ? bank.rdAllowedAt :
                                                        bank.wrAllowedAt;

            DPRINTF(DRAM, "%s checking DRAM packet in bank %d, row %d\n",
                    __func__, pkt->bank, pkt->row);

            // check if rank is not doing a refresh and thus is available,
            // if not, jump to the next packet
            if (burstReady(pkt)) {

                DPRINTF(DRAM,
                        "%s bank %d - Rank %d available\n", __func__,
                        pkt->bank, pkt->rank);

                // check if it is a row hit
                if (bank.openRow == pkt->row) {
                    // no additional rank-to-rank or same bank-group
                    // delays, or we switched read/write and might as well
                    // go for the row hit
                    if (col_allowed_at <= min_col_at) {
                        // FCFS within the hits, giving priority to
                        // commands that can issue seamlessly, without
                        // additional delay, such as same rank accesses
                        // and/or different bank-group accesses
                        DPRINTF(DRAM, "%s Seamless buffer hit\n", __func__);
                        selected_pkt_it = i;
                        selected_col_at = col_allowed_at;
                        // no need to look through the remaining queue entries
                        break;
                    } else if (!found_hidden_bank && !found_prepped_pkt) {
                        // if we did not find a packet to a closed row that can
                        // issue the bank commands without incurring delay, and
                        // did not yet find a packet to a prepped row, remember
                        // the current one
                        selected_pkt_it = i;
                        selected_col_at = col_allowed_at;
                        found_prepped_pkt = true;
                        DPRINTF(DRAM, "%s Prepped row buffer hit\n", __func__);
                    }
                } else if (!found_earliest_pkt) {
                    // if we have not initialised the bank status, do it
                    // now, and only once per scheduling decisions
                    if (!filled_earliest_banks) {
                        // determine entries with earliest bank delay
                        std::tie(earliest_banks, hidden_bank_prep) =
                            minBankPrep(queue, min_col_at);
                        filled_earliest_banks = true;
                    }

                    // bank is amongst first available banks
                    // minBankPrep will give priority to packets that can
                    // issue seamlessly
                    if (bits(earliest_banks[pkt->rank],
                             pkt->bank, pkt->bank)) {
                        found_earliest_pkt = true;
                        found_hidden_bank = hidden_bank_prep;

                        // give priority to packets that can issue
                        // bank commands 'behind the scenes'
                        // any additional delay if any will be due to
                        // col-to-col command requirements
                        if (hidden_bank_prep || !found_prepped_pkt) {
                            selected_pkt_it = i;
                            selected_col_at = col_allowed_at;
                        }
                    }
                }
            } else {
                DPRINTF(DRAM, "%s bank %d - Rank %d not available\n", __func__,
                        pkt->bank, pkt->rank);
            }
        }
    }

    if (selected_pkt_it == queue.end()) {
        DPRINTF(DRAM, "%s no available DRAM ranks found\n", __func__);
    }

    return std::make_pair(selected_pkt_it, selected_col_at);
}

void
DRAMInterface::activateBank(Rank& rank_ref, Bank& bank_ref,
                       Tick act_tick, uint32_t row)
{
    assert(rank_ref.actTicks.size() == activationLimit);

    // verify that we have command bandwidth to issue the activate
    // if not, shift to next burst window
    Tick act_at;
    if (twoCycleActivate)
        act_at = ctrl->verifyMultiCmd(act_tick, maxCommandsPerWindow, tAAD);
    else
        act_at = ctrl->verifySingleCmd(act_tick, maxCommandsPerWindow, true);

    DPRINTF(DRAM, "Activate at tick %d\n", act_at);

    // update the open row
    assert(bank_ref.openRow == Bank::NO_ROW);
    bank_ref.openRow = row;

    // start counting anew, this covers both the case when we
    // auto-precharged, and when this access is forced to
    // precharge
    bank_ref.bytesAccessed = 0;
    bank_ref.rowAccesses = 0;

    ++rank_ref.numBanksActive;
    assert(rank_ref.numBanksActive <= banksPerRank);

    DPRINTF(DRAM, "Activate bank %d, rank %d at tick %lld, now got "
            "%d active\n", bank_ref.bank, rank_ref.rank, act_at,
            ranks[rank_ref.rank]->numBanksActive);

    rank_ref.cmdList.push_back(Command(MemCommand::ACT, bank_ref.bank,
                               act_at));

    DPRINTF(DRAMPower, "%llu,ACT,%d,%d\n", divCeil(act_at, tCK) -
            timeStampOffset, bank_ref.bank, rank_ref.rank);

    // The next access has to respect tRAS for this bank
    bank_ref.preAllowedAt = act_at + tRAS;

    // Respect the row-to-column command delay for both read and write cmds
    bank_ref.rdAllowedAt = std::max(act_at + tRCD_RD, bank_ref.rdAllowedAt);
    bank_ref.wrAllowedAt = std::max(act_at + tRCD_WR, bank_ref.wrAllowedAt);

    // start by enforcing tRRD
    for (int i = 0; i < banksPerRank; i++) {
        // next activate to any bank in this rank must not happen
        // before tRRD
        if (bankGroupArch && (bank_ref.bankgr == rank_ref.banks[i].bankgr)) {
            // bank group architecture requires longer delays between
            // ACT commands within the same bank group.  Use tRRD_L
            // in this case
            rank_ref.banks[i].actAllowedAt = std::max(act_at + tRRD_L,
                                             rank_ref.banks[i].actAllowedAt);
        } else {
            // use shorter tRRD value when either
            // 1) bank group architecture is not supportted
            // 2) bank is in a different bank group
            rank_ref.banks[i].actAllowedAt = std::max(act_at + tRRD,
                                             rank_ref.banks[i].actAllowedAt);
        }
    }

    // next, we deal with tXAW, if the activation limit is disabled
    // then we directly schedule an activate power event
    if (!rank_ref.actTicks.empty()) {
        // sanity check
        if (rank_ref.actTicks.back() &&
           (act_at - rank_ref.actTicks.back()) < tXAW) {
            panic("Got %d activates in window %d (%llu - %llu) which "
                  "is smaller than %llu\n", activationLimit, act_at -
                  rank_ref.actTicks.back(), act_at,
                  rank_ref.actTicks.back(), tXAW);
        }

        // shift the times used for the book keeping, the last element
        // (highest index) is the oldest one and hence the lowest value
        rank_ref.actTicks.pop_back();

        // record an new activation (in the future)
        rank_ref.actTicks.push_front(act_at);

        // cannot activate more than X times in time window tXAW, push the
        // next one (the X + 1'st activate) to be tXAW away from the
        // oldest in our window of X
        if (rank_ref.actTicks.back() &&
           (act_at - rank_ref.actTicks.back()) < tXAW) {
            DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate "
                    "no earlier than %llu\n", activationLimit,
                    rank_ref.actTicks.back() + tXAW);
            for (int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                rank_ref.banks[j].actAllowedAt =
                    std::max(rank_ref.actTicks.back() + tXAW,
                             rank_ref.banks[j].actAllowedAt);
        }
    }

    // at the point when this activate takes place, make sure we
    // transition to the active power state
    if (!rank_ref.activateEvent.scheduled())
        schedule(rank_ref.activateEvent, act_at);
    else if (rank_ref.activateEvent.when() > act_at)
        // move it sooner in time
        reschedule(rank_ref.activateEvent, act_at);
}

void
DRAMInterface::prechargeBank(Rank& rank_ref, Bank& bank, Tick pre_tick,
                             bool auto_or_preall, bool trace)
{
    // make sure the bank has an open row
    assert(bank.openRow != Bank::NO_ROW);

    // sample the bytes per activate here since we are closing
    // the page
    stats.bytesPerActivate.sample(bank.bytesAccessed);

    bank.openRow = Bank::NO_ROW;

    Tick pre_at = pre_tick;
    if (auto_or_preall) {
        // no precharge allowed before this one
        bank.preAllowedAt = pre_at;
    } else {
        // Issuing an explicit PRE command
        // Verify that we have command bandwidth to issue the precharge
        // if not, shift to next burst window
        pre_at = ctrl->verifySingleCmd(pre_tick, maxCommandsPerWindow, true);
        // enforce tPPD
        for (int i = 0; i < banksPerRank; i++) {
            rank_ref.banks[i].preAllowedAt = std::max(pre_at + tPPD,
                                             rank_ref.banks[i].preAllowedAt);
        }
    }

    Tick pre_done_at = pre_at + tRP;

    bank.actAllowedAt = std::max(bank.actAllowedAt, pre_done_at);

    assert(rank_ref.numBanksActive != 0);
    --rank_ref.numBanksActive;

    DPRINTF(DRAM, "Precharging bank %d, rank %d at tick %lld, now got "
            "%d active\n", bank.bank, rank_ref.rank, pre_at,
            rank_ref.numBanksActive);

    if (trace) {

        rank_ref.cmdList.push_back(Command(MemCommand::PRE, bank.bank,
                                   pre_at));
        DPRINTF(DRAMPower, "%llu,PRE,%d,%d\n", divCeil(pre_at, tCK) -
                timeStampOffset, bank.bank, rank_ref.rank);
    }

    // if we look at the current number of active banks we might be
    // tempted to think the DRAM is now idle, however this can be
    // undone by an activate that is scheduled to happen before we
    // would have reached the idle state, so schedule an event and
    // rather check once we actually make it to the point in time when
    // the (last) precharge takes place
    if (!rank_ref.prechargeEvent.scheduled()) {
        schedule(rank_ref.prechargeEvent, pre_done_at);
        // New event, increment count
        ++rank_ref.outstandingEvents;
    } else if (rank_ref.prechargeEvent.when() < pre_done_at) {
        reschedule(rank_ref.prechargeEvent, pre_done_at);
    }
}

std::pair<Tick, Tick>
DRAMInterface::doBurstAccess(MemPacket* mem_pkt, Tick next_burst_at,
                             const std::vector<MemPacketQueue>& queue)
{
    DPRINTF(DRAM, "Timing access to addr %#x, rank/bank/row %d %d %d\n",
            mem_pkt->addr, mem_pkt->rank, mem_pkt->bank, mem_pkt->row);

    // get the rank
    Rank& rank_ref = *ranks[mem_pkt->rank];

    assert(rank_ref.inRefIdleState());

    // are we in or transitioning to a low-power state and have not scheduled
    // a power-up event?
    // if so, wake up from power down to issue RD/WR burst
    if (rank_ref.inLowPowerState) {
        assert(rank_ref.pwrState != PWR_SREF);
        rank_ref.scheduleWakeUpEvent(tXP);
    }

    // get the bank
    Bank& bank_ref = rank_ref.banks[mem_pkt->bank];

    // for the state we need to track if it is a row hit or not
    bool row_hit = true;

    // Determine the access latency and update the bank state
    if (bank_ref.openRow == mem_pkt->row) {
        // nothing to do
    } else {
        row_hit = false;

        // If there is a page open, precharge it.
        if (bank_ref.openRow != Bank::NO_ROW) {
            prechargeBank(rank_ref, bank_ref, std::max(bank_ref.preAllowedAt,
                                                   curTick()));
        }

        // next we need to account for the delay in activating the page
        Tick act_tick = std::max(bank_ref.actAllowedAt, curTick());

        // Record the activation and deal with all the global timing
        // constraints caused be a new activation (tRRD and tXAW)
        activateBank(rank_ref, bank_ref, act_tick, mem_pkt->row);
    }

    // respect any constraints on the command (e.g. tRCD or tCCD)
    const Tick col_allowed_at = mem_pkt->isRead() ?
                                bank_ref.rdAllowedAt : bank_ref.wrAllowedAt;

    // we need to wait until the bus is available before we can issue
    // the command; need to ensure minimum bus delay requirement is met
    Tick cmd_at = std::max({col_allowed_at, next_burst_at, curTick()});

    // verify that we have command bandwidth to issue the burst
    // if not, shift to next burst window
    Tick max_sync = clkResyncDelay + (mem_pkt->isRead() ? tRL : tWL);
    if (dataClockSync && ((cmd_at - rank_ref.lastBurstTick) > max_sync))
        cmd_at = ctrl->verifyMultiCmd(cmd_at, maxCommandsPerWindow, tCK);
    else
        cmd_at = ctrl->verifySingleCmd(cmd_at, maxCommandsPerWindow, false);

    // if we are interleaving bursts, ensure that
    // 1) we don't double interleave on next burst issue
    // 2) we are at an interleave boundary; if not, shift to next boundary
    Tick burst_gap = tBURST_MIN;
    if (burstInterleave) {
        if (cmd_at == (rank_ref.lastBurstTick + tBURST_MIN)) {
            // already interleaving, push next command to end of full burst
            burst_gap = tBURST;
        } else if (cmd_at < (rank_ref.lastBurstTick + tBURST)) {
            // not at an interleave boundary after bandwidth check
            // Shift command to tBURST boundary to avoid data contention
            // Command will remain in the same burst window given that
            // tBURST is less than tBURST_MAX
            cmd_at = rank_ref.lastBurstTick + tBURST;
        }
    }
    DPRINTF(DRAM, "Schedule RD/WR burst at tick %d\n", cmd_at);

    // update the packet ready time
    if (mem_pkt->isRead()) {
        mem_pkt->readyTime = cmd_at + tRL + tBURST;
    } else {
        mem_pkt->readyTime = cmd_at + tWL + tBURST;
    }

    rank_ref.lastBurstTick = cmd_at;

    // update the time for the next read/write burst for each
    // bank (add a max with tCCD/tCCD_L/tCCD_L_WR here)
    Tick dly_to_rd_cmd;
    Tick dly_to_wr_cmd;
    for (int j = 0; j < ranksPerChannel; j++) {
        for (int i = 0; i < banksPerRank; i++) {
            if (mem_pkt->rank == j) {
                if (bankGroupArch &&
                   (bank_ref.bankgr == ranks[j]->banks[i].bankgr)) {
                    // bank group architecture requires longer delays between
                    // RD/WR burst commands to the same bank group.
                    // tCCD_L is default requirement for same BG timing
                    // tCCD_L_WR is required for write-to-write
                    // Need to also take bus turnaround delays into account
                    dly_to_rd_cmd = mem_pkt->isRead() ?
                                    tCCD_L : std::max(tCCD_L, wrToRdDlySameBG);
                    dly_to_wr_cmd = mem_pkt->isRead() ?
                                    std::max(tCCD_L, rdToWrDlySameBG) :
                                    tCCD_L_WR;
                } else {
                    // tBURST is default requirement for diff BG timing
                    // Need to also take bus turnaround delays into account
                    dly_to_rd_cmd = mem_pkt->isRead() ? burst_gap :
                                                       writeToReadDelay();
                    dly_to_wr_cmd = mem_pkt->isRead() ? readToWriteDelay() :
                                                       burst_gap;
                }
            } else {
                // different rank is by default in a different bank group and
                // doesn't require longer tCCD or additional RTW, WTR delays
                // Need to account for rank-to-rank switching
                dly_to_wr_cmd = rankToRankDelay();
                dly_to_rd_cmd = rankToRankDelay();
            }
            ranks[j]->banks[i].rdAllowedAt = std::max(cmd_at + dly_to_rd_cmd,
                                             ranks[j]->banks[i].rdAllowedAt);
            ranks[j]->banks[i].wrAllowedAt = std::max(cmd_at + dly_to_wr_cmd,
                                             ranks[j]->banks[i].wrAllowedAt);
        }
    }

    // Save rank of current access
    activeRank = mem_pkt->rank;

    // If this is a write, we also need to respect the write recovery
    // time before a precharge, in the case of a read, respect the
    // read to precharge constraint
    bank_ref.preAllowedAt = std::max(bank_ref.preAllowedAt,
                                 mem_pkt->isRead() ? cmd_at + tRTP :
                                 mem_pkt->readyTime + tWR);

    // increment the bytes accessed and the accesses per row
    bank_ref.bytesAccessed += burstSize;
    ++bank_ref.rowAccesses;

    // if we reached the max, then issue with an auto-precharge
    bool auto_precharge = pageMgmt == enums::close ||
        bank_ref.rowAccesses == maxAccessesPerRow;

    // if we did not hit the limit, we might still want to
    // auto-precharge
    if (!auto_precharge &&
        (pageMgmt == enums::open_adaptive ||
         pageMgmt == enums::close_adaptive)) {
        // a twist on the open and close page policies:
        // 1) open_adaptive page policy does not blindly keep the
        // page open, but close it if there are no row hits, and there
        // are bank conflicts in the queue
        // 2) close_adaptive page policy does not blindly close the
        // page, but closes it only if there are no row hits in the queue.
        // In this case, only force an auto precharge when there
        // are no same page hits in the queue
        bool got_more_hits = false;
        bool got_bank_conflict = false;

        for (uint8_t i = 0; i < ctrl->numPriorities(); ++i) {
            auto p = queue[i].begin();
            // keep on looking until we find a hit or reach the end of the
            // queue
            // 1) if a hit is found, then both open and close adaptive
            //    policies keep the page open
            // 2) if no hit is found, got_bank_conflict is set to true if a
            //    bank conflict request is waiting in the queue
            // 3) make sure we are not considering the packet that we are
            //    currently dealing with
            while (!got_more_hits && p != queue[i].end()) {

                if ((*p)->pseudoChannel != pseudoChannel) {
                    // only consider if this pkt belongs to this interface
                    ++p;
                    continue;
                }

                if (mem_pkt != (*p)) {
                    bool same_rank_bank = (mem_pkt->rank == (*p)->rank) &&
                                          (mem_pkt->bank == (*p)->bank);

                    bool same_row = mem_pkt->row == (*p)->row;
                    got_more_hits |= same_rank_bank && same_row;
                    got_bank_conflict |= same_rank_bank && !same_row;
                }
                ++p;
            }

            if (got_more_hits)
                break;
        }

        // auto pre-charge when either
        // 1) open_adaptive policy, we have not got any more hits, and
        //    have a bank conflict
        // 2) close_adaptive policy and we have not got any more hits
        auto_precharge = !got_more_hits &&
            (got_bank_conflict || pageMgmt == enums::close_adaptive);
    }

    // DRAMPower trace command to be written
    std::string mem_cmd = mem_pkt->isRead() ? "RD" : "WR";

    // MemCommand required for DRAMPower library
    MemCommand::cmds command = (mem_cmd == "RD") ? MemCommand::RD :
                                                   MemCommand::WR;

    rank_ref.cmdList.push_back(Command(command, mem_pkt->bank, cmd_at));

    DPRINTF(DRAMPower, "%llu,%s,%d,%d\n", divCeil(cmd_at, tCK) -
            timeStampOffset, mem_cmd, mem_pkt->bank, mem_pkt->rank);

    // if this access should use auto-precharge, then we are
    // closing the row after the read/write burst
    if (auto_precharge) {
        // if auto-precharge push a PRE command at the correct tick to the
        // list used by DRAMPower library to calculate power
        prechargeBank(rank_ref, bank_ref, std::max(curTick(),
                      bank_ref.preAllowedAt), true);

        DPRINTF(DRAM, "Auto-precharged bank: %d\n", mem_pkt->bankId);
    }

    // Update the stats and schedule the next request
    if (mem_pkt->isRead()) {
        // Every respQueue which will generate an event, increment count
        ++rank_ref.outstandingEvents;

        stats.readBursts++;
        if (row_hit)
            stats.readRowHits++;
        stats.bytesRead += burstSize;
        stats.perBankRdBursts[mem_pkt->bankId]++;

        // Update latency stats
        stats.totMemAccLat += mem_pkt->readyTime - mem_pkt->entryTime;
        stats.totQLat += cmd_at - mem_pkt->entryTime;
        stats.totBusLat += tBURST;
    } else {
        // Schedule write done event to decrement event count
        // after the readyTime has been reached
        // Only schedule latest write event to minimize events
        // required; only need to ensure that final event scheduled covers
        // the time that writes are outstanding and bus is active
        // to holdoff power-down entry events
        if (!rank_ref.writeDoneEvent.scheduled()) {
            schedule(rank_ref.writeDoneEvent, mem_pkt->readyTime);
            // New event, increment count
            ++rank_ref.outstandingEvents;

        } else if (rank_ref.writeDoneEvent.when() < mem_pkt->readyTime) {
            reschedule(rank_ref.writeDoneEvent, mem_pkt->readyTime);
        }
        // will remove write from queue when returned to parent function
        // decrement count for DRAM rank
        --rank_ref.writeEntries;

        stats.writeBursts++;
        if (row_hit)
            stats.writeRowHits++;
        stats.bytesWritten += burstSize;
        stats.perBankWrBursts[mem_pkt->bankId]++;

    }
    // Update bus state to reflect when previous command was issued
    return std::make_pair(cmd_at, cmd_at + burst_gap);
}

void
DRAMInterface::addRankToRankDelay(Tick cmd_at)
{
    // update timing for DRAM ranks due to bursts issued
    // to ranks on other media interfaces
    for (auto n : ranks) {
        for (int i = 0; i < banksPerRank; i++) {
            // different rank by default
            // Need to only account for rank-to-rank switching
            n->banks[i].rdAllowedAt = std::max(cmd_at + rankToRankDelay(),
                                             n->banks[i].rdAllowedAt);
            n->banks[i].wrAllowedAt = std::max(cmd_at + rankToRankDelay(),
                                             n->banks[i].wrAllowedAt);
        }
    }
}

DRAMInterface::DRAMInterface(const DRAMInterfaceParams &_p)
    : MemInterface(_p),
      bankGroupsPerRank(_p.bank_groups_per_rank),
      bankGroupArch(_p.bank_groups_per_rank > 0),
      tRL(_p.tCL),
      tWL(_p.tCWL),
      tBURST_MIN(_p.tBURST_MIN), tBURST_MAX(_p.tBURST_MAX),
      tCCD_L_WR(_p.tCCD_L_WR), tCCD_L(_p.tCCD_L),
      tRCD_RD(_p.tRCD), tRCD_WR(_p.tRCD_WR),
      tRP(_p.tRP), tRAS(_p.tRAS), tWR(_p.tWR), tRTP(_p.tRTP),
      tRFC(_p.tRFC), tREFI(_p.tREFI), tRRD(_p.tRRD), tRRD_L(_p.tRRD_L),
      tPPD(_p.tPPD), tAAD(_p.tAAD),
      tXAW(_p.tXAW), tXP(_p.tXP), tXS(_p.tXS),
      clkResyncDelay(_p.tBURST_MAX),
      dataClockSync(_p.data_clock_sync),
      burstInterleave(tBURST != tBURST_MIN),
      twoCycleActivate(_p.two_cycle_activate),
      activationLimit(_p.activation_limit),
      wrToRdDlySameBG(tWL + _p.tBURST_MAX + _p.tWTR_L),
      rdToWrDlySameBG(_p.tRTW + _p.tBURST_MAX),
      pageMgmt(_p.page_policy),
      maxAccessesPerRow(_p.max_accesses_per_row),
      timeStampOffset(0), activeRank(0),
      enableDRAMPowerdown(_p.enable_dram_powerdown),
      lastStatsResetTick(0),
      stats(*this)
{
    DPRINTF(DRAM, "Setting up DRAM Interface\n");

    fatal_if(!isPowerOf2(burstSize), "DRAM burst size %d is not allowed, "
             "must be a power of two\n", burstSize);

    // sanity check the ranks since we rely on bit slicing for the
    // address decoding
    fatal_if(!isPowerOf2(ranksPerChannel), "DRAM rank count of %d is "
             "not allowed, must be a power of two\n", ranksPerChannel);

    for (int i = 0; i < ranksPerChannel; i++) {
        DPRINTF(DRAM, "Creating DRAM rank %d \n", i);
        Rank* rank = new Rank(_p, i, *this);
        ranks.push_back(rank);
    }

    // determine the dram actual capacity from the DRAM config in Mbytes
    uint64_t deviceCapacity = deviceSize / (1024 * 1024) * devicesPerRank *
                              ranksPerChannel;

    uint64_t capacity = 1ULL << ceilLog2(AbstractMemory::size());

    DPRINTF(DRAM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());

    // if actual DRAM size does not match memory capacity in system warn!
    if (deviceCapacity != capacity / (1024 * 1024))
        warn("DRAM device capacity (%d Mbytes) does not match the "
             "address range assigned (%d Mbytes)\n", deviceCapacity,
             capacity / (1024 * 1024));

    DPRINTF(DRAM, "Row buffer size %d bytes with %d bursts per row buffer\n",
            rowBufferSize, burstsPerRowBuffer);

    rowsPerBank = capacity / (rowBufferSize * banksPerRank * ranksPerChannel);

    // some basic sanity checks
    if (tREFI <= tRP || tREFI <= tRFC) {
        fatal("tREFI (%d) must be larger than tRP (%d) and tRFC (%d)\n",
              tREFI, tRP, tRFC);
    }

    // basic bank group architecture checks ->
    if (bankGroupArch) {
        // must have at least one bank per bank group
        if (bankGroupsPerRank > banksPerRank) {
            fatal("banks per rank (%d) must be equal to or larger than "
                  "banks groups per rank (%d)\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // must have same number of banks in each bank group
        if ((banksPerRank % bankGroupsPerRank) != 0) {
            fatal("Banks per rank (%d) must be evenly divisible by bank "
                  "groups per rank (%d) for equal banks per bank group\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // tCCD_L should be greater than minimal, back-to-back burst delay
        if (tCCD_L <= tBURST) {
            fatal("tCCD_L (%d) should be larger than the minimum bus delay "
                  "(%d) when bank groups per rank (%d) is greater than 1\n",
                  tCCD_L, tBURST, bankGroupsPerRank);
        }
        // tCCD_L_WR should be greater than minimal, back-to-back burst delay
        if (tCCD_L_WR <= tBURST) {
            fatal("tCCD_L_WR (%d) should be larger than the minimum bus delay "
                  " (%d) when bank groups per rank (%d) is greater than 1\n",
                  tCCD_L_WR, tBURST, bankGroupsPerRank);
        }
        // tRRD_L is greater than minimal, same bank group ACT-to-ACT delay
        // some datasheets might specify it equal to tRRD
        if (tRRD_L < tRRD) {
            fatal("tRRD_L (%d) should be larger than tRRD (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tRRD_L, tRRD, bankGroupsPerRank);
        }
    }
}

void
DRAMInterface::init()
{
    AbstractMemory::init();

    // a bit of sanity checks on the interleaving, save it for here to
    // ensure that the system pointer is initialised
    if (range.interleaved()) {
        if (addrMapping == enums::RoRaBaChCo) {
            if (rowBufferSize != range.granularity()) {
                fatal("Channel interleaving of %s doesn't match RoRaBaChCo "
                      "address map\n", name());
            }
        } else if (addrMapping == enums::RoRaBaCoCh ||
                   addrMapping == enums::RoCoRaBaCh) {
            // for the interleavings with channel bits in the bottom,
            // if the system uses a channel striping granularity that
            // is larger than the DRAM burst size, then map the
            // sequential accesses within a stripe to a number of
            // columns in the DRAM, effectively placing some of the
            // lower-order column bits as the least-significant bits
            // of the address (above the ones denoting the burst size)
            assert(burstsPerStripe >= 1);

            // channel striping has to be done at a granularity that
            // is equal or larger to a cache line
            if (system()->cacheLineSize() > range.granularity()) {
                fatal("Channel interleaving of %s must be at least as large "
                      "as the cache line size\n", name());
            }

            // ...and equal or smaller than the row-buffer size
            if (rowBufferSize < range.granularity()) {
                fatal("Channel interleaving of %s must be at most as large "
                      "as the row-buffer size\n", name());
            }
            // this is essentially the check above, so just to be sure
            assert(burstsPerStripe <= burstsPerRowBuffer);
        }
    }
}

void
DRAMInterface::startup()
{
    if (system()->isTimingMode()) {
        // timestamp offset should be in clock cycles for DRAMPower
        timeStampOffset = divCeil(curTick(), tCK);

        for (auto r : ranks) {
            r->startup(curTick() + tREFI - tRP);
        }
    }
}

bool
DRAMInterface::isBusy(bool read_queue_empty, bool all_writes_nvm)
{
    int busy_ranks = 0;
    for (auto r : ranks) {
        if (!r->inRefIdleState()) {
            if (r->pwrState != PWR_SREF) {
                // rank is busy refreshing
                DPRINTF(DRAMState, "Rank %d is not available\n", r->rank);
                busy_ranks++;

                // let the rank know that if it was waiting to drain, it
                // is now done and ready to proceed
                r->checkDrainDone();
            }

            // check if we were in self-refresh and haven't started
            // to transition out
            if ((r->pwrState == PWR_SREF) && r->inLowPowerState) {
                DPRINTF(DRAMState, "Rank %d is in self-refresh\n", r->rank);
                // if we have commands queued to this rank and we don't have
                // a minimum number of active commands enqueued,
                // exit self-refresh
                if (r->forceSelfRefreshExit()) {
                    DPRINTF(DRAMState, "rank %d was in self refresh and"
                           " should wake up\n", r->rank);
                    //wake up from self-refresh
                    r->scheduleWakeUpEvent(tXS);
                    // things are brought back into action once a refresh is
                    // performed after self-refresh
                    // continue with selection for other ranks
                }
            }
        }
    }
    return (busy_ranks == ranksPerChannel);
}

MemPacket*
DRAMInterface::decodePacket(const PacketPtr pkt, Addr pkt_addr,
                       unsigned size, bool is_read, uint8_t pseudo_channel)
{
    // decode the address based on the address mapping scheme, with
    // Ro, Ra, Co, Ba and Ch denoting row, rank, column, bank and
    // channel, respectively
    uint8_t rank;
    uint8_t bank;
    // use a 64-bit unsigned during the computations as the row is
    // always the top bits, and check before creating the packet
    uint64_t row;

    // Get packed address, starting at 0
    Addr addr = getCtrlAddr(pkt_addr);

    // truncate the address to a memory burst, which makes it unique to
    // a specific buffer, row, bank, rank and channel
    addr = addr / burstSize;

    // we have removed the lowest order address bits that denote the
    // position within the column
    if (addrMapping == enums::RoRaBaChCo || addrMapping == enums::RoRaBaCoCh) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / burstsPerRowBuffer;

        // after the channel bits, get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else if (addrMapping == enums::RoCoRaBaCh) {
        // with emerging technologies, could have small page size with
        // interleaving granularity greater than row buffer
        if (burstsPerStripe > burstsPerRowBuffer) {
            // remove column bits which are a subset of burstsPerStripe
            addr = addr / burstsPerRowBuffer;
        } else {
            // remove lower column bits below channel bits
            addr = addr / burstsPerStripe;
        }

        // start with the bank bits, as this provides the maximum
        // opportunity for parallelism between requests
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // next get the rank bits
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // next, the higher-order column bites
        if (burstsPerStripe < burstsPerRowBuffer) {
            addr = addr / (burstsPerRowBuffer / burstsPerStripe);
        }

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);
    assert(row < Bank::NO_ROW);

    DPRINTF(DRAM, "Address: %#x Rank %d Bank %d Row %d\n",
            pkt_addr, rank, bank, row);

    // create the corresponding memory packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rank + bank;

    return new MemPacket(pkt, is_read, true, pseudo_channel, rank, bank, row,
                   bank_id, pkt_addr, size);
}

void DRAMInterface::setupRank(const uint8_t rank, const bool is_read)
{
    // increment entry count of the rank based on packet type
    if (is_read) {
        ++ranks[rank]->readEntries;
    } else {
        ++ranks[rank]->writeEntries;
    }
}

void
DRAMInterface::respondEvent(uint8_t rank)
{
    Rank& rank_ref = *ranks[rank];

    // if a read has reached its ready-time, decrement the number of reads
    // At this point the packet has been handled and there is a possibility
    // to switch to low-power mode if no other packet is available
    --rank_ref.readEntries;
    DPRINTF(DRAM, "number of read entries for rank %d is %d\n",
            rank, rank_ref.readEntries);

    // counter should at least indicate one outstanding request
    // for this read
    assert(rank_ref.outstandingEvents > 0);
    // read response received, decrement count
    --rank_ref.outstandingEvents;

    // at this moment should not have transitioned to a low-power state
    assert((rank_ref.pwrState != PWR_SREF) &&
           (rank_ref.pwrState != PWR_PRE_PDN) &&
           (rank_ref.pwrState != PWR_ACT_PDN));

    // track if this is the last packet before idling
    // and that there are no outstanding commands to this rank
    if (rank_ref.isQueueEmpty() && rank_ref.outstandingEvents == 0 &&
        rank_ref.inRefIdleState() && enableDRAMPowerdown) {
        // verify that there are no events scheduled
        assert(!rank_ref.activateEvent.scheduled());
        assert(!rank_ref.prechargeEvent.scheduled());

        // if coming from active state, schedule power event to
        // active power-down else go to precharge power-down
        DPRINTF(DRAMState, "Rank %d sleep at tick %d; current power state is "
                "%d\n", rank, curTick(), rank_ref.pwrState);

        // default to ACT power-down unless already in IDLE state
        // could be in IDLE if PRE issued before data returned
        PowerState next_pwr_state = PWR_ACT_PDN;
        if (rank_ref.pwrState == PWR_IDLE) {
            next_pwr_state = PWR_PRE_PDN;
        }

        rank_ref.powerDownSleep(next_pwr_state, curTick());
    }
}

void
DRAMInterface::checkRefreshState(uint8_t rank)
{
    Rank& rank_ref = *ranks[rank];

    if ((rank_ref.refreshState == REF_PRE) &&
        !rank_ref.prechargeEvent.scheduled()) {
          // kick the refresh event loop into action again if banks already
          // closed and just waiting for read to complete
          schedule(rank_ref.refreshEvent, curTick());
    }
}

void
DRAMInterface::drainRanks()
{
    // also need to kick off events to exit self-refresh
    for (auto r : ranks) {
        // force self-refresh exit, which in turn will issue auto-refresh
        if (r->pwrState == PWR_SREF) {
            DPRINTF(DRAM,"Rank%d: Forcing self-refresh wakeup in drain\n",
                    r->rank);
            r->scheduleWakeUpEvent(tXS);
        }
    }
}

bool
DRAMInterface::allRanksDrained() const
{
    // true until proven false
    bool all_ranks_drained = true;
    for (auto r : ranks) {
        // then verify that the power state is IDLE ensuring all banks are
        // closed and rank is not in a low power state. Also verify that rank
        // is idle from a refresh point of view.
        all_ranks_drained = r->inPwrIdleState() && r->inRefIdleState() &&
            all_ranks_drained;
    }
    return all_ranks_drained;
}

void
DRAMInterface::suspend()
{
    for (auto r : ranks) {
        r->suspend();
    }
}

std::pair<std::vector<uint32_t>, bool>
DRAMInterface::minBankPrep(const MemPacketQueue& queue,
                      Tick min_col_at) const
{
    Tick min_act_at = MaxTick;
    std::vector<uint32_t> bank_mask(ranksPerChannel, 0);

    // Flag condition when burst can issue back-to-back with previous burst
    bool found_seamless_bank = false;

    // Flag condition when bank can be opened without incurring additional
    // delay on the data bus
    bool hidden_bank_prep = false;

    // determine if we have queued transactions targetting the
    // bank in question
    std::vector<bool> got_waiting(ranksPerChannel * banksPerRank, false);
    for (const auto& p : queue) {
        if (p->pseudoChannel != pseudoChannel)
            continue;
        if (p->isDram() && ranks[p->rank]->inRefIdleState())
            got_waiting[p->bankId] = true;
    }

    // Find command with optimal bank timing
    // Will prioritize commands that can issue seamlessly.
    for (int i = 0; i < ranksPerChannel; i++) {
        for (int j = 0; j < banksPerRank; j++) {
            uint16_t bank_id = i * banksPerRank + j;

            // if we have waiting requests for the bank, and it is
            // amongst the first available, update the mask
            if (got_waiting[bank_id]) {
                // make sure this rank is not currently refreshing.
                assert(ranks[i]->inRefIdleState());
                // simplistic approximation of when the bank can issue
                // an activate, ignoring any rank-to-rank switching
                // cost in this calculation
                Tick act_at = ranks[i]->banks[j].openRow == Bank::NO_ROW ?
                    std::max(ranks[i]->banks[j].actAllowedAt, curTick()) :
                    std::max(ranks[i]->banks[j].preAllowedAt, curTick()) + tRP;

                // latest Tick for which ACT can occur without
                // incurring additoinal delay on the data bus
                const Tick tRCD = ctrl->inReadBusState(false, this) ?
                                                tRCD_RD : tRCD_WR;
                const Tick hidden_act_max =
                            std::max(min_col_at - tRCD, curTick());

                // When is the earliest the R/W burst can issue?
                const Tick col_allowed_at = ctrl->inReadBusState(false,
                                              this) ?
                                              ranks[i]->banks[j].rdAllowedAt :
                                              ranks[i]->banks[j].wrAllowedAt;
                Tick col_at = std::max(col_allowed_at, act_at + tRCD);

                // bank can issue burst back-to-back (seamlessly) with
                // previous burst
                bool new_seamless_bank = col_at <= min_col_at;

                // if we found a new seamless bank or we have no
                // seamless banks, and got a bank with an earlier
                // activate time, it should be added to the bit mask
                if (new_seamless_bank ||
                    (!found_seamless_bank && act_at <= min_act_at)) {
                    // if we did not have a seamless bank before, and
                    // we do now, reset the bank mask, also reset it
                    // if we have not yet found a seamless bank and
                    // the activate time is smaller than what we have
                    // seen so far
                    if (!found_seamless_bank &&
                        (new_seamless_bank || act_at < min_act_at)) {
                        std::fill(bank_mask.begin(), bank_mask.end(), 0);
                    }

                    found_seamless_bank |= new_seamless_bank;

                    // ACT can occur 'behind the scenes'
                    hidden_bank_prep = act_at <= hidden_act_max;

                    // set the bit corresponding to the available bank
                    replaceBits(bank_mask[i], j, j, 1);
                    min_act_at = act_at;
                }
            }
        }
    }

    return std::make_pair(bank_mask, hidden_bank_prep);
}

DRAMInterface::Rank::Rank(const DRAMInterfaceParams &_p,
                         int _rank, DRAMInterface& _dram)
    : EventManager(&_dram), dram(_dram),
      pwrStateTrans(PWR_IDLE), pwrStatePostRefresh(PWR_IDLE),
      pwrStateTick(0), refreshDueAt(0), pwrState(PWR_IDLE),
      refreshState(REF_IDLE), inLowPowerState(false), rank(_rank),
      readEntries(0), writeEntries(0), outstandingEvents(0),
      wakeUpAllowedAt(0), power(_p, false), banks(_p.banks_per_rank),
      numBanksActive(0), actTicks(_p.activation_limit, 0), lastBurstTick(0),
      writeDoneEvent([this]{ processWriteDoneEvent(); }, name()),
      activateEvent([this]{ processActivateEvent(); }, name()),
      prechargeEvent([this]{ processPrechargeEvent(); }, name()),
      refreshEvent([this]{ processRefreshEvent(); }, name()),
      powerEvent([this]{ processPowerEvent(); }, name()),
      wakeUpEvent([this]{ processWakeUpEvent(); }, name()),
      stats(_dram, *this)
{
    for (int b = 0; b < _p.banks_per_rank; b++) {
        banks[b].bank = b;
        // GDDR addressing of banks to BG is linear.
        // Here we assume that all DRAM generations address bank groups as
        // follows:
        if (_p.bank_groups_per_rank > 0) {
            // Simply assign lower bits to bank group in order to
            // rotate across bank groups as banks are incremented
            // e.g. with 4 banks per bank group and 16 banks total:
            //    banks 0,4,8,12  are in bank group 0
            //    banks 1,5,9,13  are in bank group 1
            //    banks 2,6,10,14 are in bank group 2
            //    banks 3,7,11,15 are in bank group 3
            banks[b].bankgr = b % _p.bank_groups_per_rank;
        } else {
            // No bank groups; simply assign to bank number
            banks[b].bankgr = b;
        }
    }
}

void
DRAMInterface::Rank::startup(Tick ref_tick)
{
    assert(ref_tick > curTick());

    pwrStateTick = curTick();

    // kick off the refresh, and give ourselves enough time to
    // precharge
    schedule(refreshEvent, ref_tick);
}

void
DRAMInterface::Rank::suspend()
{
    deschedule(refreshEvent);

    // Update the stats
    updatePowerStats();

    // don't automatically transition back to LP state after next REF
    pwrStatePostRefresh = PWR_IDLE;
}

bool
DRAMInterface::Rank::isQueueEmpty() const
{
    // check commmands in Q based on current bus direction
    bool no_queued_cmds = (dram.ctrl->inReadBusState(true, &(this->dram))
                          && (readEntries == 0)) ||
                          (dram.ctrl->inWriteBusState(true, &(this->dram))
                          && (writeEntries == 0));
    return no_queued_cmds;
}

void
DRAMInterface::Rank::checkDrainDone()
{
    // if this rank was waiting to drain it is now able to proceed to
    // precharge
    if (refreshState == REF_DRAIN) {
        DPRINTF(DRAM, "Refresh drain done, now precharging\n");

        refreshState = REF_PD_EXIT;

        // hand control back to the refresh event loop
        schedule(refreshEvent, curTick());
    }
}

void
DRAMInterface::Rank::flushCmdList()
{
    // at the moment sort the list of commands and update the counters
    // for DRAMPower libray when doing a refresh
    sort(cmdList.begin(), cmdList.end(), DRAMInterface::sortTime);

    auto next_iter = cmdList.begin();
    // push to commands to DRAMPower
    for ( ; next_iter != cmdList.end() ; ++next_iter) {
         Command cmd = *next_iter;
         if (cmd.timeStamp <= curTick()) {
             // Move all commands at or before curTick to DRAMPower
             power.powerlib.doCommand(cmd.type, cmd.bank,
                                      divCeil(cmd.timeStamp, dram.tCK) -
                                      dram.timeStampOffset);
         } else {
             // done - found all commands at or before curTick()
             // next_iter references the 1st command after curTick
             break;
         }
    }
    // reset cmdList to only contain commands after curTick
    // if there are no commands after curTick, updated cmdList will be empty
    // in this case, next_iter is cmdList.end()
    cmdList.assign(next_iter, cmdList.end());
}

void
DRAMInterface::Rank::processActivateEvent()
{
    // we should transition to the active state as soon as any bank is active
    if (pwrState != PWR_ACT)
        // note that at this point numBanksActive could be back at
        // zero again due to a precharge scheduled in the future
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMInterface::Rank::processPrechargeEvent()
{
    // counter should at least indicate one outstanding request
    // for this precharge
    assert(outstandingEvents > 0);
    // precharge complete, decrement count
    --outstandingEvents;

    // if we reached zero, then special conditions apply as we track
    // if all banks are precharged for the power models
    if (numBanksActive == 0) {
        // no reads to this rank in the Q and no pending
        // RD/WR or refresh commands
        if (isQueueEmpty() && outstandingEvents == 0 &&
            dram.enableDRAMPowerdown) {
            // should still be in ACT state since bank still open
            assert(pwrState == PWR_ACT);

            // All banks closed - switch to precharge power down state.
            DPRINTF(DRAMState, "Rank %d sleep at tick %d\n",
                    rank, curTick());
            powerDownSleep(PWR_PRE_PDN, curTick());
        } else {
            // we should transition to the idle state when the last bank
            // is precharged
            schedulePowerEvent(PWR_IDLE, curTick());
        }
    }
}

void
DRAMInterface::Rank::processWriteDoneEvent()
{
    // counter should at least indicate one outstanding request
    // for this write
    assert(outstandingEvents > 0);
    // Write transfer on bus has completed
    // decrement per rank counter
    --outstandingEvents;
}

void
DRAMInterface::Rank::processRefreshEvent()
{
    // when first preparing the refresh, remember when it was due
    if ((refreshState == REF_IDLE) || (refreshState == REF_SREF_EXIT)) {
        // remember when the refresh is due
        refreshDueAt = curTick();

        // proceed to drain
        refreshState = REF_DRAIN;

        // make nonzero while refresh is pending to ensure
        // power down and self-refresh are not entered
        ++outstandingEvents;

        DPRINTF(DRAM, "Refresh due\n");
    }

    // let any scheduled read or write to the same rank go ahead,
    // after which it will
    // hand control back to this event loop
    if (refreshState == REF_DRAIN) {
        // if a request is at the moment being handled and this request is
        // accessing the current rank then wait for it to finish
        if ((rank == dram.activeRank)
            && (dram.ctrl->requestEventScheduled(dram.pseudoChannel))) {
            // hand control over to the request loop until it is
            // evaluated next
            DPRINTF(DRAM, "Refresh awaiting draining\n");
            return;
        } else {
            refreshState = REF_PD_EXIT;
        }
    }

    // at this point, ensure that rank is not in a power-down state
    if (refreshState == REF_PD_EXIT) {
        // if rank was sleeping and we have't started exit process,
        // wake-up for refresh
        if (inLowPowerState) {
            DPRINTF(DRAM, "Wake Up for refresh\n");
            // save state and return after refresh completes
            scheduleWakeUpEvent(dram.tXP);
            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    // at this point, ensure that all banks are precharged
    if (refreshState == REF_PRE) {
        // precharge any active bank
        if (numBanksActive != 0) {
            // at the moment, we use a precharge all even if there is
            // only a single bank open
            DPRINTF(DRAM, "Precharging all\n");

            // first determine when we can precharge
            Tick pre_at = curTick();

            for (auto &b : banks) {
                // respect both causality and any existing bank
                // constraints, some banks could already have a
                // (auto) precharge scheduled
                pre_at = std::max(b.preAllowedAt, pre_at);
            }

            // make sure all banks per rank are precharged, and for those that
            // already are, update their availability
            Tick act_allowed_at = pre_at + dram.tRP;

            for (auto &b : banks) {
                if (b.openRow != Bank::NO_ROW) {
                    dram.prechargeBank(*this, b, pre_at, true, false);
                } else {
                    b.actAllowedAt = std::max(b.actAllowedAt, act_allowed_at);
                    b.preAllowedAt = std::max(b.preAllowedAt, pre_at);
                }
            }

            // precharge all banks in rank
            cmdList.push_back(Command(MemCommand::PREA, 0, pre_at));

            DPRINTF(DRAMPower, "%llu,PREA,0,%d\n",
                    divCeil(pre_at, dram.tCK) -
                            dram.timeStampOffset, rank);
        } else if ((pwrState == PWR_IDLE) && (outstandingEvents == 1))  {
            // Banks are closed, have transitioned to IDLE state, and
            // no outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            DPRINTF(DRAM, "All banks already precharged, starting refresh\n");

            // go ahead and kick the power state machine into gear since
            // we are already idle
            schedulePowerEvent(PWR_REF, curTick());
        } else {
            // banks state is closed but haven't transitioned pwrState to IDLE
            // or have outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            // should have outstanding precharge or read response event
            assert(prechargeEvent.scheduled() ||
                   dram.ctrl->respondEventScheduled(dram.pseudoChannel));
            // will start refresh when pwrState transitions to IDLE
        }

        assert(numBanksActive == 0);

        // wait for all banks to be precharged or read to complete
        // When precharge commands are done, power state machine will
        // transition to the idle state, and automatically move to a
        // refresh, at that point it will also call this method to get
        // the refresh event loop going again
        // Similarly, when read response completes, if all banks are
        // precharged, will call this method to get loop re-started
        return;
    }

    // last but not least we perform the actual refresh
    if (refreshState == REF_START) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick ref_done_at = curTick() + dram.tRFC;

        for (auto &b : banks) {
            b.actAllowedAt = ref_done_at;
        }

        // at the moment this affects all ranks
        cmdList.push_back(Command(MemCommand::REF, 0, curTick()));

        // Update the stats
        updatePowerStats();

        DPRINTF(DRAMPower, "%llu,REF,0,%d\n", divCeil(curTick(), dram.tCK) -
                dram.timeStampOffset, rank);

        // Update for next refresh
        refreshDueAt += dram.tREFI;

        // make sure we did not wait so long that we cannot make up
        // for it
        if (refreshDueAt < ref_done_at) {
            fatal("Refresh was delayed so long we cannot catch up\n");
        }

        // Run the refresh and schedule event to transition power states
        // when refresh completes
        refreshState = REF_RUN;
        schedule(refreshEvent, ref_done_at);
        return;
    }

    if (refreshState == REF_RUN) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        assert(!powerEvent.scheduled());

        if ((dram.ctrl->drainState() == DrainState::Draining) ||
            (dram.ctrl->drainState() == DrainState::Drained)) {
            // if draining, do not re-enter low-power mode.
            // simply go to IDLE and wait
            schedulePowerEvent(PWR_IDLE, curTick());
        } else {
            // At the moment, we sleep when the refresh ends and wait to be
            // woken up again if previously in a low-power state.
            if (pwrStatePostRefresh != PWR_IDLE) {
                // power State should be power Refresh
                assert(pwrState == PWR_REF);
                DPRINTF(DRAMState, "Rank %d sleeping after refresh and was in "
                        "power state %d before refreshing\n", rank,
                        pwrStatePostRefresh);
                powerDownSleep(pwrState, curTick());

            // Force PRE power-down if there are no outstanding commands
            // in Q after refresh.
            } else if (isQueueEmpty() && dram.enableDRAMPowerdown) {
                // still have refresh event outstanding but there should
                // be no other events outstanding
                assert(outstandingEvents == 1);
                DPRINTF(DRAMState, "Rank %d sleeping after refresh but was NOT"
                        " in a low power state before refreshing\n", rank);
                powerDownSleep(PWR_PRE_PDN, curTick());

            } else {
                // move to the idle power state once the refresh is done, this
                // will also move the refresh state machine to the refresh
                // idle state
                schedulePowerEvent(PWR_IDLE, curTick());
            }
        }

        // At this point, we have completed the current refresh.
        // In the SREF bypass case, we do not get to this state in the
        // refresh STM and therefore can always schedule next event.
        // Compensate for the delay in actually performing the refresh
        // when scheduling the next one
        schedule(refreshEvent, refreshDueAt - dram.tRP);

        DPRINTF(DRAMState, "Refresh done at %llu and next refresh"
                " at %llu\n", curTick(), refreshDueAt);
    }
}

void
DRAMInterface::Rank::schedulePowerEvent(PowerState pwr_state, Tick tick)
{
    // respect causality
    assert(tick >= curTick());

    if (!powerEvent.scheduled()) {
        DPRINTF(DRAMState, "Scheduling power event at %llu to state %d\n",
                tick, pwr_state);

        // insert the new transition
        pwrStateTrans = pwr_state;

        schedule(powerEvent, tick);
    } else {
        panic("Scheduled power event at %llu to state %d, "
              "with scheduled event at %llu to %d\n", tick, pwr_state,
              powerEvent.when(), pwrStateTrans);
    }
}

void
DRAMInterface::Rank::powerDownSleep(PowerState pwr_state, Tick tick)
{
    // if low power state is active low, schedule to active low power state.
    // in reality tCKE is needed to enter active low power. This is neglected
    // here and could be added in the future.
    if (pwr_state == PWR_ACT_PDN) {
        schedulePowerEvent(pwr_state, tick);
        // push command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_ACT, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_ACT,0,%d\n", divCeil(tick,
                dram.tCK) - dram.timeStampOffset, rank);
    } else if (pwr_state == PWR_PRE_PDN) {
        // if low power state is precharge low, schedule to precharge low
        // power state. In reality tCKE is needed to enter active low power.
        // This is neglected here.
        schedulePowerEvent(pwr_state, tick);
        //push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                dram.tCK) - dram.timeStampOffset, rank);
    } else if (pwr_state == PWR_REF) {
        // if a refresh just occurred
        // transition to PRE_PDN now that all banks are closed
        // precharge power down requires tCKE to enter. For simplicity
        // this is not considered.
        schedulePowerEvent(PWR_PRE_PDN, tick);
        //push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                dram.tCK) - dram.timeStampOffset, rank);
    } else if (pwr_state == PWR_SREF) {
        // should only enter SREF after PRE-PD wakeup to do a refresh
        assert(pwrStatePostRefresh == PWR_PRE_PDN);
        // self refresh requires time tCKESR to enter. For simplicity,
        // this is not considered.
        schedulePowerEvent(PWR_SREF, tick);
        // push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::SREN, 0, tick));
        DPRINTF(DRAMPower, "%llu,SREN,0,%d\n", divCeil(tick,
                dram.tCK) - dram.timeStampOffset, rank);
    }
    // Ensure that we don't power-down and back up in same tick
    // Once we commit to PD entry, do it and wait for at least 1tCK
    // This could be replaced with tCKE if/when that is added to the model
    wakeUpAllowedAt = tick + dram.tCK;

    // Transitioning to a low power state, set flag
    inLowPowerState = true;
}

void
DRAMInterface::Rank::scheduleWakeUpEvent(Tick exit_delay)
{
    Tick wake_up_tick = std::max(curTick(), wakeUpAllowedAt);

    DPRINTF(DRAMState, "Scheduling wake-up for rank %d at tick %d\n",
            rank, wake_up_tick);

    // if waking for refresh, hold previous state
    // else reset state back to IDLE
    if (refreshState == REF_PD_EXIT) {
        pwrStatePostRefresh = pwrState;
    } else {
        // don't automatically transition back to LP state after next REF
        pwrStatePostRefresh = PWR_IDLE;
    }

    // schedule wake-up with event to ensure entry has completed before
    // we try to wake-up
    schedule(wakeUpEvent, wake_up_tick);

    for (auto &b : banks) {
        // respect both causality and any existing bank
        // constraints, some banks could already have a
        // (auto) precharge scheduled
        b.wrAllowedAt = std::max(wake_up_tick + exit_delay, b.wrAllowedAt);
        b.rdAllowedAt = std::max(wake_up_tick + exit_delay, b.rdAllowedAt);
        b.preAllowedAt = std::max(wake_up_tick + exit_delay, b.preAllowedAt);
        b.actAllowedAt = std::max(wake_up_tick + exit_delay, b.actAllowedAt);
    }
    // Transitioning out of low power state, clear flag
    inLowPowerState = false;

    // push to DRAMPower
    // use pwrStateTrans for cases where we have a power event scheduled
    // to enter low power that has not yet been processed
    if (pwrStateTrans == PWR_ACT_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_ACT, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_ACT,0,%d\n", divCeil(wake_up_tick,
                dram.tCK) - dram.timeStampOffset, rank);

    } else if (pwrStateTrans == PWR_PRE_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_PRE, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_PRE,0,%d\n", divCeil(wake_up_tick,
                dram.tCK) - dram.timeStampOffset, rank);
    } else if (pwrStateTrans == PWR_SREF) {
        cmdList.push_back(Command(MemCommand::SREX, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,SREX,0,%d\n", divCeil(wake_up_tick,
                dram.tCK) - dram.timeStampOffset, rank);
    }
}

void
DRAMInterface::Rank::processWakeUpEvent()
{
    // Should be in a power-down or self-refresh state
    assert((pwrState == PWR_ACT_PDN) || (pwrState == PWR_PRE_PDN) ||
           (pwrState == PWR_SREF));

    // Check current state to determine transition state
    if (pwrState == PWR_ACT_PDN) {
        // banks still open, transition to PWR_ACT
        schedulePowerEvent(PWR_ACT, curTick());
    } else {
        // transitioning from a precharge power-down or self-refresh state
        // banks are closed - transition to PWR_IDLE
        schedulePowerEvent(PWR_IDLE, curTick());
    }
}

void
DRAMInterface::Rank::processPowerEvent()
{
    assert(curTick() >= pwrStateTick);
    // remember where we were, and for how long
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;

    // update the accounting
    stats.pwrStateTime[prev_state] += duration;

    // track to total idle time
    if ((prev_state == PWR_PRE_PDN) || (prev_state == PWR_ACT_PDN) ||
        (prev_state == PWR_SREF)) {
        stats.totalIdleTime += duration;
    }

    pwrState = pwrStateTrans;
    pwrStateTick = curTick();

    // if rank was refreshing, make sure to start scheduling requests again
    if (prev_state == PWR_REF) {
        // bus IDLED prior to REF
        // counter should be one for refresh command only
        assert(outstandingEvents == 1);
        // REF complete, decrement count and go back to IDLE
        --outstandingEvents;
        refreshState = REF_IDLE;

        DPRINTF(DRAMState, "Was refreshing for %llu ticks\n", duration);
        // if moving back to power-down after refresh
        if (pwrState != PWR_IDLE) {
            assert(pwrState == PWR_PRE_PDN);
            DPRINTF(DRAMState, "Switching to power down state after refreshing"
                    " rank %d at %llu tick\n", rank, curTick());
        }

        // completed refresh event, ensure next request is scheduled
        if (!(dram.ctrl->requestEventScheduled(dram.pseudoChannel))) {
            DPRINTF(DRAM, "Scheduling next request after refreshing"
                           " rank %d, PC %d \n", rank, dram.pseudoChannel);
            dram.ctrl->restartScheduler(curTick(), dram.pseudoChannel);
        }
    }

    if ((pwrState == PWR_ACT) && (refreshState == REF_PD_EXIT)) {
        // have exited ACT PD
        assert(prev_state == PWR_ACT_PDN);

        // go back to REF event and close banks
        refreshState = REF_PRE;
        schedule(refreshEvent, curTick());
    } else if (pwrState == PWR_IDLE) {
        DPRINTF(DRAMState, "All banks precharged\n");
        if (prev_state == PWR_SREF) {
            // set refresh state to REF_SREF_EXIT, ensuring inRefIdleState
            // continues to return false during tXS after SREF exit
            // Schedule a refresh which kicks things back into action
            // when it finishes
            refreshState = REF_SREF_EXIT;
            schedule(refreshEvent, curTick() + dram.tXS);
        } else {
            // if we have a pending refresh, and are now moving to
            // the idle state, directly transition to, or schedule refresh
            if ((refreshState == REF_PRE) || (refreshState == REF_PD_EXIT)) {
                // ensure refresh is restarted only after final PRE command.
                // do not restart refresh if controller is in an intermediate
                // state, after PRE_PDN exit, when banks are IDLE but an
                // ACT is scheduled.
                if (!activateEvent.scheduled()) {
                    // there should be nothing waiting at this point
                    assert(!powerEvent.scheduled());
                    if (refreshState == REF_PD_EXIT) {
                        // exiting PRE PD, will be in IDLE until tXP expires
                        // and then should transition to PWR_REF state
                        assert(prev_state == PWR_PRE_PDN);
                        schedulePowerEvent(PWR_REF, curTick() + dram.tXP);
                    } else if (refreshState == REF_PRE) {
                        // can directly move to PWR_REF state and proceed below
                        pwrState = PWR_REF;
                    }
                } else {
                    // must have PRE scheduled to transition back to IDLE
                    // and re-kick off refresh
                    assert(prechargeEvent.scheduled());
                }
            }
        }
    }

    // transition to the refresh state and re-start refresh process
    // refresh state machine will schedule the next power state transition
    if (pwrState == PWR_REF) {
        // completed final PRE for refresh or exiting power-down
        assert(refreshState == REF_PRE || refreshState == REF_PD_EXIT);

        // exited PRE PD for refresh, with no pending commands
        // bypass auto-refresh and go straight to SREF, where memory
        // will issue refresh immediately upon entry
        if (pwrStatePostRefresh == PWR_PRE_PDN && isQueueEmpty() &&
           (dram.ctrl->drainState() != DrainState::Draining) &&
           (dram.ctrl->drainState() != DrainState::Drained) &&
           dram.enableDRAMPowerdown) {
            DPRINTF(DRAMState, "Rank %d bypassing refresh and transitioning "
                    "to self refresh at %11u tick\n", rank, curTick());
            powerDownSleep(PWR_SREF, curTick());

            // Since refresh was bypassed, remove event by decrementing count
            assert(outstandingEvents == 1);
            --outstandingEvents;

            // reset state back to IDLE temporarily until SREF is entered
            pwrState = PWR_IDLE;

        // Not bypassing refresh for SREF entry
        } else {
            DPRINTF(DRAMState, "Refreshing\n");

            // there should be nothing waiting at this point
            assert(!powerEvent.scheduled());

            // kick the refresh event loop into action again, and that
            // in turn will schedule a transition to the idle power
            // state once the refresh is done
            schedule(refreshEvent, curTick());

            // Banks transitioned to IDLE, start REF
            refreshState = REF_START;
        }
    }

}

void
DRAMInterface::Rank::updatePowerStats()
{
    // All commands up to refresh have completed
    // flush cmdList to DRAMPower
    flushCmdList();

    // Call the function that calculates window energy at intermediate update
    // events like at refresh, stats dump as well as at simulation exit.
    // Window starts at the last time the calcWindowEnergy function was called
    // and is upto current time.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), dram.tCK) -
                                    dram.timeStampOffset);

    // Get the energy from DRAMPower
    Data::MemoryPowerModel::Energy energy = power.powerlib.getEnergy();

    // The energy components inside the power lib are calculated over
    // the window so accumulate into the corresponding gem5 stat
    stats.actEnergy += energy.act_energy * dram.devicesPerRank;
    stats.preEnergy += energy.pre_energy * dram.devicesPerRank;
    stats.readEnergy += energy.read_energy * dram.devicesPerRank;
    stats.writeEnergy += energy.write_energy * dram.devicesPerRank;
    stats.refreshEnergy += energy.ref_energy * dram.devicesPerRank;
    stats.actBackEnergy += energy.act_stdby_energy * dram.devicesPerRank;
    stats.preBackEnergy += energy.pre_stdby_energy * dram.devicesPerRank;
    stats.actPowerDownEnergy += energy.f_act_pd_energy * dram.devicesPerRank;
    stats.prePowerDownEnergy += energy.f_pre_pd_energy * dram.devicesPerRank;
    stats.selfRefreshEnergy += energy.sref_energy * dram.devicesPerRank;

    // Accumulate window energy into the total energy.
    stats.totalEnergy += energy.window_energy * dram.devicesPerRank;
    // Average power must not be accumulated but calculated over the time
    // since last stats reset. sim_clock::Frequency is tick period not tick
    // frequency.
    //              energy (pJ)     1e-9
    // power (mW) = ----------- * ----------
    //              time (tick)   tick_frequency
    stats.averagePower = (stats.totalEnergy.value() /
                    (curTick() - dram.lastStatsResetTick)) *
                    (sim_clock::Frequency / 1000000000.0);
}

void
DRAMInterface::Rank::computeStats()
{
    DPRINTF(DRAM,"Computing stats due to a dump callback\n");

    // Update the stats
    updatePowerStats();

    // final update of power state times
    stats.pwrStateTime[pwrState] += (curTick() - pwrStateTick);
    pwrStateTick = curTick();
}

void
DRAMInterface::Rank::resetStats() {
    // The only way to clear the counters in DRAMPower is to call
    // calcWindowEnergy function as that then calls clearCounters. The
    // clearCounters method itself is private.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), dram.tCK) -
                                    dram.timeStampOffset);

}

bool
DRAMInterface::Rank::forceSelfRefreshExit() const {
    return (readEntries != 0) ||
           (dram.ctrl->inWriteBusState(true, &(this->dram))
            && (writeEntries != 0));
}

void
DRAMInterface::DRAMStats::resetStats()
{
    dram.lastStatsResetTick = curTick();
}

DRAMInterface::DRAMStats::DRAMStats(DRAMInterface &_dram)
    : statistics::Group(&_dram),
    dram(_dram),

    ADD_STAT(readBursts, statistics::units::Count::get(),
             "Number of DRAM read bursts"),
    ADD_STAT(writeBursts, statistics::units::Count::get(),
             "Number of DRAM write bursts"),

    ADD_STAT(perBankRdBursts, statistics::units::Count::get(),
             "Per bank write bursts"),
    ADD_STAT(perBankWrBursts, statistics::units::Count::get(),
             "Per bank write bursts"),

    ADD_STAT(totQLat, statistics::units::Tick::get(),
             "Total ticks spent queuing"),
    ADD_STAT(totBusLat, statistics::units::Tick::get(),
             "Total ticks spent in databus transfers"),
    ADD_STAT(totMemAccLat, statistics::units::Tick::get(),
             "Total ticks spent from burst creation until serviced "
             "by the DRAM"),

    ADD_STAT(avgQLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average queueing delay per DRAM burst"),
    ADD_STAT(avgBusLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average bus latency per DRAM burst"),
    ADD_STAT(avgMemAccLat, statistics::units::Rate<
                statistics::units::Tick, statistics::units::Count>::get(),
             "Average memory access latency per DRAM burst"),

    ADD_STAT(readRowHits, statistics::units::Count::get(),
             "Number of row buffer hits during reads"),
    ADD_STAT(writeRowHits, statistics::units::Count::get(),
             "Number of row buffer hits during writes"),
    ADD_STAT(readRowHitRate, statistics::units::Ratio::get(),
             "Row buffer hit rate for reads"),
    ADD_STAT(writeRowHitRate, statistics::units::Ratio::get(),
             "Row buffer hit rate for writes"),

    ADD_STAT(bytesPerActivate, statistics::units::Byte::get(),
             "Bytes accessed per row activation"),
    ADD_STAT(bytesRead, statistics::units::Byte::get(),
            "Total bytes read"),
    ADD_STAT(bytesWritten, statistics::units::Byte::get(),
            "Total bytes written"),

    ADD_STAT(avgRdBW, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average DRAM read bandwidth in MiBytes/s"),
    ADD_STAT(avgWrBW, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Average DRAM write bandwidth in MiBytes/s"),
    ADD_STAT(peakBW,  statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Theoretical peak bandwidth in MiByte/s"),

    ADD_STAT(busUtil, statistics::units::Ratio::get(),
             "Data bus utilization in percentage"),
    ADD_STAT(busUtilRead, statistics::units::Ratio::get(),
             "Data bus utilization in percentage for reads"),
    ADD_STAT(busUtilWrite, statistics::units::Ratio::get(),
             "Data bus utilization in percentage for writes"),

    ADD_STAT(pageHitRate, statistics::units::Ratio::get(),
             "Row buffer hit rate, read and write combined")

{
}

void
DRAMInterface::DRAMStats::regStats()
{
    using namespace statistics;

    avgQLat.precision(2);
    avgBusLat.precision(2);
    avgMemAccLat.precision(2);

    readRowHitRate.precision(2);
    writeRowHitRate.precision(2);

    perBankRdBursts.init(dram.banksPerRank * dram.ranksPerChannel);
    perBankWrBursts.init(dram.banksPerRank * dram.ranksPerChannel);

    bytesPerActivate
        .init(dram.maxAccessesPerRow ?
              dram.maxAccessesPerRow : dram.rowBufferSize)
        .flags(nozero);

    peakBW.precision(2);
    busUtil.precision(2);
    busUtilWrite.precision(2);
    busUtilRead.precision(2);

    pageHitRate.precision(2);

    // Formula stats
    avgQLat = totQLat / readBursts;
    avgBusLat = totBusLat / readBursts;
    avgMemAccLat = totMemAccLat / readBursts;

    readRowHitRate = (readRowHits / readBursts) * 100;
    writeRowHitRate = (writeRowHits / writeBursts) * 100;

    avgRdBW = (bytesRead / 1000000) / simSeconds;
    avgWrBW = (bytesWritten / 1000000) / simSeconds;
    peakBW = (sim_clock::Frequency / dram.burstDelay()) *
              dram.bytesPerBurst() / 1000000;

    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;
    busUtilRead = avgRdBW / peakBW * 100;
    busUtilWrite = avgWrBW / peakBW * 100;

    pageHitRate = (writeRowHits + readRowHits) /
        (writeBursts + readBursts) * 100;
}

DRAMInterface::RankStats::RankStats(DRAMInterface &_dram, Rank &_rank)
    : statistics::Group(&_dram, csprintf("rank%d", _rank.rank).c_str()),
    rank(_rank),

    ADD_STAT(actEnergy, statistics::units::Joule::get(),
             "Energy for activate commands per rank (pJ)"),
    ADD_STAT(preEnergy, statistics::units::Joule::get(),
             "Energy for precharge commands per rank (pJ)"),
    ADD_STAT(readEnergy, statistics::units::Joule::get(),
             "Energy for read commands per rank (pJ)"),
    ADD_STAT(writeEnergy, statistics::units::Joule::get(),
             "Energy for write commands per rank (pJ)"),
    ADD_STAT(refreshEnergy, statistics::units::Joule::get(),
             "Energy for refresh commands per rank (pJ)"),
    ADD_STAT(actBackEnergy, statistics::units::Joule::get(),
             "Energy for active background per rank (pJ)"),
    ADD_STAT(preBackEnergy, statistics::units::Joule::get(),
             "Energy for precharge background per rank (pJ)"),
    ADD_STAT(actPowerDownEnergy, statistics::units::Joule::get(),
             "Energy for active power-down per rank (pJ)"),
    ADD_STAT(prePowerDownEnergy, statistics::units::Joule::get(),
             "Energy for precharge power-down per rank (pJ)"),
    ADD_STAT(selfRefreshEnergy, statistics::units::Joule::get(),
             "Energy for self refresh per rank (pJ)"),

    ADD_STAT(totalEnergy, statistics::units::Joule::get(),
             "Total energy per rank (pJ)"),
    ADD_STAT(averagePower, statistics::units::Watt::get(),
             "Core power per rank (mW)"),

    ADD_STAT(totalIdleTime, statistics::units::Tick::get(),
             "Total Idle time Per DRAM Rank"),
    ADD_STAT(pwrStateTime, statistics::units::Tick::get(),
             "Time in different power states")
{
}

void
DRAMInterface::RankStats::regStats()
{
    statistics::Group::regStats();

    pwrStateTime
        .init(6)
        .subname(0, "IDLE")
        .subname(1, "REF")
        .subname(2, "SREF")
        .subname(3, "PRE_PDN")
        .subname(4, "ACT")
        .subname(5, "ACT_PDN");
}

void
DRAMInterface::RankStats::resetStats()
{
    statistics::Group::resetStats();

    rank.resetStats();
}

void
DRAMInterface::RankStats::preDumpStats()
{
    statistics::Group::preDumpStats();

    rank.computeStats();
}

} // namespace memory
} // namespace gem5
