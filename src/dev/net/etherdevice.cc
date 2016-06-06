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
 *
 * Authors: Nathan Binkert
 *          Lisa Hsu
 */

#include "dev/net/etherdevice.hh"

#include "sim/stats.hh"

void
EtherDevice::regStats()
{
    PciDevice::regStats();

    txBytes
        .name(name() + ".txBytes")
        .desc("Bytes Transmitted")
        .prereq(txBytes)
        ;

    rxBytes
        .name(name() + ".rxBytes")
        .desc("Bytes Received")
        .prereq(rxBytes)
        ;

    txPackets
        .name(name() + ".txPackets")
        .desc("Number of Packets Transmitted")
        .prereq(txBytes)
        ;

    rxPackets
        .name(name() + ".rxPackets")
        .desc("Number of Packets Received")
        .prereq(rxBytes)
        ;

    txIpChecksums
        .name(name() + ".txIpChecksums")
        .desc("Number of tx IP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    rxIpChecksums
        .name(name() + ".rxIpChecksums")
        .desc("Number of rx IP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    txTcpChecksums
        .name(name() + ".txTcpChecksums")
        .desc("Number of tx TCP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    rxTcpChecksums
        .name(name() + ".rxTcpChecksums")
        .desc("Number of rx TCP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    txUdpChecksums
        .name(name() + ".txUdpChecksums")
        .desc("Number of tx UDP Checksums done by device")
        .precision(0)
        .prereq(txBytes)
        ;

    rxUdpChecksums
        .name(name() + ".rxUdpChecksums")
        .desc("Number of rx UDP Checksums done by device")
        .precision(0)
        .prereq(rxBytes)
        ;

    descDmaReads
        .name(name() + ".descDMAReads")
        .desc("Number of descriptors the device read w/ DMA")
        .precision(0)
        ;

    descDmaWrites
        .name(name() + ".descDMAWrites")
        .desc("Number of descriptors the device wrote w/ DMA")
        .precision(0)
        ;

    descDmaRdBytes
        .name(name() + ".descDmaReadBytes")
        .desc("number of descriptor bytes read w/ DMA")
        .precision(0)
        ;

    descDmaWrBytes
        .name(name() + ".descDmaWriteBytes")
        .desc("number of descriptor bytes write w/ DMA")
        .precision(0)
        ;

    txBandwidth
        .name(name() + ".txBandwidth")
        .desc("Transmit Bandwidth (bits/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    rxBandwidth
        .name(name() + ".rxBandwidth")
        .desc("Receive Bandwidth (bits/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    totBandwidth
        .name(name() + ".totBandwidth")
        .desc("Total Bandwidth (bits/s)")
        .precision(0)
        .prereq(totBytes)
        ;

    totPackets
        .name(name() + ".totPackets")
        .desc("Total Packets")
        .precision(0)
        .prereq(totBytes)
        ;

    totBytes
        .name(name() + ".totBytes")
        .desc("Total Bytes")
        .precision(0)
        .prereq(totBytes)
        ;

    totPacketRate
        .name(name() + ".totPPS")
        .desc("Total Tranmission Rate (packets/s)")
        .precision(0)
        .prereq(totBytes)
        ;

    txPacketRate
        .name(name() + ".txPPS")
        .desc("Packet Tranmission Rate (packets/s)")
        .precision(0)
        .prereq(txBytes)
        ;

    rxPacketRate
        .name(name() + ".rxPPS")
        .desc("Packet Reception Rate (packets/s)")
        .precision(0)
        .prereq(rxBytes)
        ;

    postedSwi
        .name(name() + ".postedSwi")
        .desc("number of software interrupts posted to CPU")
        .precision(0)
        ;

    totalSwi
        .name(name() + ".totalSwi")
        .desc("total number of Swi written to ISR")
        .precision(0)
        ;

    coalescedSwi
        .name(name() + ".coalescedSwi")
        .desc("average number of Swi's coalesced into each post")
        .precision(0)
        ;

    postedRxIdle
        .name(name() + ".postedRxIdle")
        .desc("number of rxIdle interrupts posted to CPU")
        .precision(0)
        ;

    totalRxIdle
        .name(name() + ".totalRxIdle")
        .desc("total number of RxIdle written to ISR")
        .precision(0)
        ;

    coalescedRxIdle
        .name(name() + ".coalescedRxIdle")
        .desc("average number of RxIdle's coalesced into each post")
        .precision(0)
        ;

    postedRxOk
        .name(name() + ".postedRxOk")
        .desc("number of RxOk interrupts posted to CPU")
        .precision(0)
        ;

    totalRxOk
        .name(name() + ".totalRxOk")
        .desc("total number of RxOk written to ISR")
        .precision(0)
        ;

    coalescedRxOk
        .name(name() + ".coalescedRxOk")
        .desc("average number of RxOk's coalesced into each post")
        .precision(0)
        ;

    postedRxDesc
        .name(name() + ".postedRxDesc")
        .desc("number of RxDesc interrupts posted to CPU")
        .precision(0)
        ;

    totalRxDesc
        .name(name() + ".totalRxDesc")
        .desc("total number of RxDesc written to ISR")
        .precision(0)
        ;

    coalescedRxDesc
        .name(name() + ".coalescedRxDesc")
        .desc("average number of RxDesc's coalesced into each post")
        .precision(0)
        ;

    postedTxOk
        .name(name() + ".postedTxOk")
        .desc("number of TxOk interrupts posted to CPU")
        .precision(0)
        ;

    totalTxOk
        .name(name() + ".totalTxOk")
        .desc("total number of TxOk written to ISR")
        .precision(0)
        ;

    coalescedTxOk
        .name(name() + ".coalescedTxOk")
        .desc("average number of TxOk's coalesced into each post")
        .precision(0)
        ;

    postedTxIdle
        .name(name() + ".postedTxIdle")
        .desc("number of TxIdle interrupts posted to CPU")
        .precision(0)
        ;

    totalTxIdle
        .name(name() + ".totalTxIdle")
        .desc("total number of TxIdle written to ISR")
        .precision(0)
        ;

    coalescedTxIdle
        .name(name() + ".coalescedTxIdle")
        .desc("average number of TxIdle's coalesced into each post")
        .precision(0)
        ;

    postedTxDesc
        .name(name() + ".postedTxDesc")
        .desc("number of TxDesc interrupts posted to CPU")
        .precision(0)
        ;

    totalTxDesc
        .name(name() + ".totalTxDesc")
        .desc("total number of TxDesc written to ISR")
        .precision(0)
        ;

    coalescedTxDesc
        .name(name() + ".coalescedTxDesc")
        .desc("average number of TxDesc's coalesced into each post")
        .precision(0)
        ;

    postedRxOrn
        .name(name() + ".postedRxOrn")
        .desc("number of RxOrn posted to CPU")
        .precision(0)
        ;

    totalRxOrn
        .name(name() + ".totalRxOrn")
        .desc("total number of RxOrn written to ISR")
        .precision(0)
        ;

    coalescedRxOrn
        .name(name() + ".coalescedRxOrn")
        .desc("average number of RxOrn's coalesced into each post")
        .precision(0)
        ;

    coalescedTotal
        .name(name() + ".coalescedTotal")
        .desc("average number of interrupts coalesced into each post")
        .precision(0)
        ;

    postedInterrupts
        .name(name() + ".postedInterrupts")
        .desc("number of posts to CPU")
        .precision(0)
        ;

    droppedPackets
        .name(name() + ".droppedPackets")
        .desc("number of packets dropped")
        .precision(0)
        ;

    coalescedSwi = totalSwi / postedInterrupts;
    coalescedRxIdle = totalRxIdle / postedInterrupts;
    coalescedRxOk = totalRxOk / postedInterrupts;
    coalescedRxDesc = totalRxDesc / postedInterrupts;
    coalescedTxOk = totalTxOk / postedInterrupts;
    coalescedTxIdle = totalTxIdle / postedInterrupts;
    coalescedTxDesc = totalTxDesc / postedInterrupts;
    coalescedRxOrn = totalRxOrn / postedInterrupts;

    coalescedTotal = (totalSwi + totalRxIdle + totalRxOk + totalRxDesc +
                      totalTxOk + totalTxIdle + totalTxDesc +
                      totalRxOrn) / postedInterrupts;

    txBandwidth = txBytes * Stats::constant(8) / simSeconds;
    rxBandwidth = rxBytes * Stats::constant(8) / simSeconds;
    totBandwidth = txBandwidth + rxBandwidth;
    totBytes = txBytes + rxBytes;
    totPackets = txPackets + rxPackets;

    txPacketRate = txPackets / simSeconds;
    rxPacketRate = rxPackets / simSeconds;
    totPacketRate = totPackets / simSeconds;
}
