/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "dev/net/etherdevice.hh"

#include "sim/stats.hh"

EtherDevice::EtherDeviceStats::EtherDeviceStats(Stats::Group *parent)
    : Stats::Group(parent, "EtherDevice"),
      ADD_STAT(postedInterrupts, UNIT_COUNT, "Number of posts to CPU"),
      ADD_STAT(txBytes, UNIT_BYTE, "Bytes Transmitted"),
      ADD_STAT(rxBytes, UNIT_BYTE, "Bytes Received"),
      ADD_STAT(txPackets, UNIT_COUNT, "Number of Packets Transmitted"),
      ADD_STAT(rxPackets, UNIT_COUNT, "Number of Packets Received"),
      ADD_STAT(txBandwidth, UNIT_RATE(Stats::Units::Bit, Stats::Units::Second),
               "Transmit Bandwidth (bits/s)",
               txBytes * Stats::constant(8) / simSeconds),
      ADD_STAT(rxBandwidth, UNIT_RATE(Stats::Units::Bit, Stats::Units::Second),
               "Receive Bandwidth (bits/s)",
               rxBytes * Stats::constant(8) / simSeconds),
      ADD_STAT(txIpChecksums, UNIT_COUNT,
               "Number of tx IP Checksums done by device"),
      ADD_STAT(rxIpChecksums, UNIT_COUNT,
               "Number of rx IP Checksums done by device"),
      ADD_STAT(txTcpChecksums, UNIT_COUNT,
               "Number of tx TCP Checksums done by device"),
      ADD_STAT(rxTcpChecksums, UNIT_COUNT,
               "Number of rx TCP Checksums done by device"),
      ADD_STAT(txUdpChecksums, UNIT_COUNT,
               "Number of tx UDP Checksums done by device"),
      ADD_STAT(rxUdpChecksums, UNIT_COUNT,
               "Number of rx UDP Checksums done by device"),
      ADD_STAT(descDmaReads, UNIT_COUNT,
               "Number of descriptors the device read w/ DMA"),
      ADD_STAT(descDmaWrites, UNIT_COUNT,
               "Number of descriptors the device wrote w/ DMA"),
      ADD_STAT(descDmaRdBytes, UNIT_COUNT,
               "Number of descriptor bytes read w/ DMA"),
      ADD_STAT(descDmaWrBytes, UNIT_COUNT,
               "Number of descriptor bytes write w/ DMA"),
      ADD_STAT(totBandwidth,
               UNIT_RATE(Stats::Units::Bit, Stats::Units::Second),
               "Total Bandwidth (bits/s)",
               txBandwidth + rxBandwidth),
      ADD_STAT(totPackets, UNIT_COUNT, "Total Packets", txPackets + rxPackets),
      ADD_STAT(totBytes, UNIT_BYTE, "Total Bytes", txBytes + rxBytes),
      ADD_STAT(totPacketRate,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Second),
               "Total Packet Tranmission Rate",
               totPackets / simSeconds),
      ADD_STAT(txPacketRate,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Second),
               "Packet Tranmission Rate",
               txPackets / simSeconds),
      ADD_STAT(rxPacketRate,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Second),
               "Packet Reception Rate",
               rxPackets / simSeconds),
      ADD_STAT(postedSwi, UNIT_COUNT,
               "Number of software interrupts posted to CPU"),
      ADD_STAT(totalSwi, UNIT_COUNT, "Total number of Swi written to ISR"),
      ADD_STAT(coalescedSwi,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of Swi's coalesced into each post",
               totalSwi / postedInterrupts),
      ADD_STAT(postedRxIdle, UNIT_COUNT,
               "Number of rxIdle interrupts posted to CPU"),
      ADD_STAT(totalRxIdle, UNIT_COUNT,
               "Total number of RxIdle written to ISR"),
      ADD_STAT(coalescedRxIdle,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of RxIdle's coalesced into each post",
               totalRxIdle / postedInterrupts),
      ADD_STAT(postedRxOk, UNIT_COUNT,
               "Number of RxOk interrupts posted to CPU"),
      ADD_STAT(totalRxOk, UNIT_COUNT, "Total number of RxOk written to ISR"),
      ADD_STAT(coalescedRxOk,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of RxOk's coalesced into each post",
               totalRxOk / postedInterrupts),
      ADD_STAT(postedRxDesc, UNIT_COUNT,
               "Number of RxDesc interrupts posted to CPU"),
      ADD_STAT(totalRxDesc, UNIT_COUNT,
               "Total number of RxDesc written to ISR"),
      ADD_STAT(coalescedRxDesc,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of RxDesc's coalesced into each post",
               totalRxDesc / postedInterrupts),
      ADD_STAT(postedTxOk, UNIT_COUNT,
               "Number of TxOk interrupts posted to CPU"),
      ADD_STAT(totalTxOk, UNIT_COUNT,
               "Total number of TxOk written to ISR"),
      ADD_STAT(coalescedTxOk,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of TxOk's coalesced into each post",
               totalTxOk / postedInterrupts),
      ADD_STAT(postedTxIdle, UNIT_COUNT,
               "Number of TxIdle interrupts posted to CPU"),
      ADD_STAT(totalTxIdle, UNIT_COUNT,
               "Total number of TxIdle written to ISR"),
      ADD_STAT(coalescedTxIdle,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of TxIdle's coalesced into each post",
               totalTxIdle / postedInterrupts),
      ADD_STAT(postedTxDesc, UNIT_COUNT,
               "Number of TxDesc interrupts posted to CPU"),
      ADD_STAT(totalTxDesc, UNIT_COUNT,
               "Total number of TxDesc written to ISR"),
      ADD_STAT(coalescedTxDesc,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of TxDesc's coalesced into each post",
               totalTxDesc / postedInterrupts),
      ADD_STAT(postedRxOrn, UNIT_COUNT,
               "Number of RxOrn posted to CPU"),
      ADD_STAT(totalRxOrn, UNIT_COUNT,
               "Total number of RxOrn written to ISR"),
      ADD_STAT(coalescedRxOrn,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of RxOrn's coalesced into each post",
               totalRxOrn / postedInterrupts),
      ADD_STAT(coalescedTotal,
               UNIT_RATE(Stats::Units::Count, Stats::Units::Count),
               "Average number of interrupts coalesced into each post"),
      ADD_STAT(droppedPackets, UNIT_COUNT, "Number of packets dropped")
{

    postedInterrupts
        .precision(0);

    txBytes
        .prereq(txBytes);

    rxBytes
        .prereq(rxBytes);

    txPackets
        .prereq(txBytes);

    rxPackets
        .prereq(rxBytes);

    txIpChecksums
        .precision(0)
        .prereq(txBytes);

    rxIpChecksums
        .precision(0)
        .prereq(rxBytes);

    txTcpChecksums
        .precision(0)
        .prereq(txBytes);

    rxTcpChecksums
        .precision(0)
        .prereq(rxBytes);

    txUdpChecksums
        .precision(0)
        .prereq(txBytes);

    rxUdpChecksums
        .precision(0)
        .prereq(rxBytes);

    descDmaReads
        .precision(0);

    descDmaWrites
        .precision(0);

    descDmaRdBytes
        .precision(0);

    descDmaWrBytes
        .precision(0);

    txBandwidth
        .precision(0)
        .prereq(txBytes)
        ;

    rxBandwidth
        .precision(0)
        .prereq(rxBytes);

    totBandwidth
        .precision(0)
        .prereq(totBytes);

    totPackets
        .precision(0)
        .prereq(totBytes);

    totBytes
        .precision(0)
        .prereq(totBytes);

    totPacketRate
        .precision(0)
        .prereq(totBytes);

    txPacketRate
        .precision(0)
        .prereq(txBytes);

    rxPacketRate
        .precision(0)
        .prereq(rxBytes);

    postedSwi
        .precision(0);

    totalSwi
        .precision(0);

    coalescedSwi
        .precision(0);

    postedRxIdle
        .precision(0);

    totalRxIdle
        .precision(0);

    coalescedRxIdle
        .precision(0);

    postedRxOk
        .precision(0);

    totalRxOk
        .precision(0);

    coalescedRxOk
        .precision(0);

    postedRxDesc
        .precision(0);

    totalRxDesc
        .precision(0);

    coalescedRxDesc
        .precision(0);

    postedTxOk
        .precision(0);

    totalTxOk
        .precision(0);

    coalescedTxOk
        .precision(0);

    postedTxIdle
        .precision(0);

    totalTxIdle
        .precision(0);

    coalescedTxIdle
        .precision(0);

    postedTxDesc
        .precision(0);

    totalTxDesc
        .precision(0);

    coalescedTxDesc
        .precision(0);

    postedRxOrn
        .precision(0);

    totalRxOrn
        .precision(0);

    coalescedRxOrn
        .precision(0);

    coalescedTotal
        .precision(0);

    droppedPackets
        .precision(0);

    coalescedTotal = (totalSwi + totalRxIdle + totalRxOk + totalRxDesc +
                      totalTxOk + totalTxIdle + totalTxDesc +
                      totalRxOrn) / postedInterrupts;
}
