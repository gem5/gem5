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

namespace gem5
{

EtherDevice::EtherDeviceStats::EtherDeviceStats(statistics::Group *parent)
    : statistics::Group(parent, "EtherDevice"),
      ADD_STAT(postedInterrupts, statistics::units::Count::get(),
               "Number of posts to CPU"),
      ADD_STAT(txBytes, statistics::units::Byte::get(), "Bytes Transmitted"),
      ADD_STAT(rxBytes, statistics::units::Byte::get(), "Bytes Received"),
      ADD_STAT(txPackets, statistics::units::Count::get(),
               "Number of Packets Transmitted"),
      ADD_STAT(rxPackets, statistics::units::Count::get(),
               "Number of Packets Received"),
      ADD_STAT(txBandwidth,
               statistics::units::Rate<statistics::units::Bit,
                                       statistics::units::Second>::get(),
               "Transmit Bandwidth",
               txBytes * statistics::constant(8) / simSeconds),
      ADD_STAT(rxBandwidth,
               statistics::units::Rate<statistics::units::Bit,
                                       statistics::units::Second>::get(),
               "Receive Bandwidth",
               rxBytes * statistics::constant(8) / simSeconds),
      ADD_STAT(txIpChecksums, statistics::units::Count::get(),
               "Number of tx IP Checksums done by device"),
      ADD_STAT(rxIpChecksums, statistics::units::Count::get(),
               "Number of rx IP Checksums done by device"),
      ADD_STAT(txTcpChecksums, statistics::units::Count::get(),
               "Number of tx TCP Checksums done by device"),
      ADD_STAT(rxTcpChecksums, statistics::units::Count::get(),
               "Number of rx TCP Checksums done by device"),
      ADD_STAT(txUdpChecksums, statistics::units::Count::get(),
               "Number of tx UDP Checksums done by device"),
      ADD_STAT(rxUdpChecksums, statistics::units::Count::get(),
               "Number of rx UDP Checksums done by device"),
      ADD_STAT(descDmaReads, statistics::units::Count::get(),
               "Number of descriptors the device read w/ DMA"),
      ADD_STAT(descDmaWrites, statistics::units::Count::get(),
               "Number of descriptors the device wrote w/ DMA"),
      ADD_STAT(descDmaRdBytes, statistics::units::Count::get(),
               "Number of descriptor bytes read w/ DMA"),
      ADD_STAT(descDmaWrBytes, statistics::units::Count::get(),
               "Number of descriptor bytes write w/ DMA"),
      ADD_STAT(totBandwidth,
               statistics::units::Rate<statistics::units::Bit,
                                       statistics::units::Second>::get(),
               "Total Bandwidth", txBandwidth + rxBandwidth),
      ADD_STAT(totPackets, statistics::units::Count::get(), "Total Packets",
               txPackets + rxPackets),
      ADD_STAT(totBytes, statistics::units::Byte::get(), "Total Bytes",
               txBytes + rxBytes),
      ADD_STAT(totPacketRate,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Second>::get(),
               "Total Packet Tranmission Rate", totPackets / simSeconds),
      ADD_STAT(txPacketRate,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Second>::get(),
               "Packet Tranmission Rate", txPackets / simSeconds),
      ADD_STAT(rxPacketRate,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Second>::get(),
               "Packet Reception Rate", rxPackets / simSeconds),
      ADD_STAT(postedSwi, statistics::units::Count::get(),
               "Number of software interrupts posted to CPU"),
      ADD_STAT(totalSwi, statistics::units::Count::get(),
               "Total number of Swi written to ISR"),
      ADD_STAT(coalescedSwi,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of Swi's coalesced into each post",
               totalSwi / postedInterrupts),
      ADD_STAT(postedRxIdle, statistics::units::Count::get(),
               "Number of rxIdle interrupts posted to CPU"),
      ADD_STAT(totalRxIdle, statistics::units::Count::get(),
               "Total number of RxIdle written to ISR"),
      ADD_STAT(coalescedRxIdle,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of RxIdle's coalesced into each post",
               totalRxIdle / postedInterrupts),
      ADD_STAT(postedRxOk, statistics::units::Count::get(),
               "Number of RxOk interrupts posted to CPU"),
      ADD_STAT(totalRxOk, statistics::units::Count::get(),
               "Total number of RxOk written to ISR"),
      ADD_STAT(coalescedRxOk,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of RxOk's coalesced into each post",
               totalRxOk / postedInterrupts),
      ADD_STAT(postedRxDesc, statistics::units::Count::get(),
               "Number of RxDesc interrupts posted to CPU"),
      ADD_STAT(totalRxDesc, statistics::units::Count::get(),
               "Total number of RxDesc written to ISR"),
      ADD_STAT(coalescedRxDesc,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of RxDesc's coalesced into each post",
               totalRxDesc / postedInterrupts),
      ADD_STAT(postedTxOk, statistics::units::Count::get(),
               "Number of TxOk interrupts posted to CPU"),
      ADD_STAT(totalTxOk, statistics::units::Count::get(),
               "Total number of TxOk written to ISR"),
      ADD_STAT(coalescedTxOk,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of TxOk's coalesced into each post",
               totalTxOk / postedInterrupts),
      ADD_STAT(postedTxIdle, statistics::units::Count::get(),
               "Number of TxIdle interrupts posted to CPU"),
      ADD_STAT(totalTxIdle, statistics::units::Count::get(),
               "Total number of TxIdle written to ISR"),
      ADD_STAT(coalescedTxIdle,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of TxIdle's coalesced into each post",
               totalTxIdle / postedInterrupts),
      ADD_STAT(postedTxDesc, statistics::units::Count::get(),
               "Number of TxDesc interrupts posted to CPU"),
      ADD_STAT(totalTxDesc, statistics::units::Count::get(),
               "Total number of TxDesc written to ISR"),
      ADD_STAT(coalescedTxDesc,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of TxDesc's coalesced into each post",
               totalTxDesc / postedInterrupts),
      ADD_STAT(postedRxOrn, statistics::units::Count::get(),
               "Number of RxOrn posted to CPU"),
      ADD_STAT(totalRxOrn, statistics::units::Count::get(),
               "Total number of RxOrn written to ISR"),
      ADD_STAT(coalescedRxOrn,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of RxOrn's coalesced into each post",
               totalRxOrn / postedInterrupts),
      ADD_STAT(coalescedTotal,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Average number of interrupts coalesced into each post"),
      ADD_STAT(droppedPackets, statistics::units::Count::get(),
               "Number of packets dropped")
{
    postedInterrupts.precision(0);

    txBytes.prereq(txBytes);

    rxBytes.prereq(rxBytes);

    txPackets.prereq(txBytes);

    rxPackets.prereq(rxBytes);

    txIpChecksums.precision(0).prereq(txBytes);

    rxIpChecksums.precision(0).prereq(rxBytes);

    txTcpChecksums.precision(0).prereq(txBytes);

    rxTcpChecksums.precision(0).prereq(rxBytes);

    txUdpChecksums.precision(0).prereq(txBytes);

    rxUdpChecksums.precision(0).prereq(rxBytes);

    descDmaReads.precision(0);

    descDmaWrites.precision(0);

    descDmaRdBytes.precision(0);

    descDmaWrBytes.precision(0);

    txBandwidth.precision(0).prereq(txBytes);

    rxBandwidth.precision(0).prereq(rxBytes);

    totBandwidth.precision(0).prereq(totBytes);

    totPackets.precision(0).prereq(totBytes);

    totBytes.precision(0).prereq(totBytes);

    totPacketRate.precision(0).prereq(totBytes);

    txPacketRate.precision(0).prereq(txBytes);

    rxPacketRate.precision(0).prereq(rxBytes);

    postedSwi.precision(0);

    totalSwi.precision(0);

    coalescedSwi.precision(0);

    postedRxIdle.precision(0);

    totalRxIdle.precision(0);

    coalescedRxIdle.precision(0);

    postedRxOk.precision(0);

    totalRxOk.precision(0);

    coalescedRxOk.precision(0);

    postedRxDesc.precision(0);

    totalRxDesc.precision(0);

    coalescedRxDesc.precision(0);

    postedTxOk.precision(0);

    totalTxOk.precision(0);

    coalescedTxOk.precision(0);

    postedTxIdle.precision(0);

    totalTxIdle.precision(0);

    coalescedTxIdle.precision(0);

    postedTxDesc.precision(0);

    totalTxDesc.precision(0);

    coalescedTxDesc.precision(0);

    postedRxOrn.precision(0);

    totalRxOrn.precision(0);

    coalescedRxOrn.precision(0);

    coalescedTotal.precision(0);

    droppedPackets.precision(0);

    coalescedTotal = (totalSwi + totalRxIdle + totalRxOk + totalRxDesc +
                      totalTxOk + totalTxIdle + totalTxDesc + totalRxOrn) /
                     postedInterrupts;
}

} // namespace gem5
