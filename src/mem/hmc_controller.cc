#include "mem/hmc_controller.hh"

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/HMCController.hh"

namespace gem5
{

HMCController::HMCController(const HMCControllerParams &p)
    : NoncoherentXBar(p),
      numMemSidePorts(p.port_mem_side_ports_connection_count),
      rr_counter(0)
{
    assert(p.port_cpu_side_ports_connection_count == 1);
}

// Since this module is a load distributor, all its request ports have the same
//  range so we should keep only one of the ranges and ignore the others
void
HMCController::recvRangeChange(PortID mem_side_port_id)
{
    if (mem_side_port_id == 0) {
        gotAllAddrRanges = true;
        BaseXBar::recvRangeChange(mem_side_port_id);
    } else
        gotAddrRanges[mem_side_port_id] = true;
}

int
HMCController::rotate_counter()
{
    int current_value = rr_counter;
    rr_counter++;
    if (rr_counter == numMemSidePorts)
        rr_counter = 0;
    return current_value;
}

bool
HMCController::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a non-coherent component
    assert(!pkt->isExpressSnoop());

    // For now, this is a simple round robin counter, for distribution the
    //  load among the serial links
    PortID mem_side_port_id = rotate_counter();

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x\n", src_port->name(),
            pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // before forwarding the packet (and possibly altering it),
    // remember if we are expecting a response
    const bool expect_response =
        pkt->needsResponse() && !pkt->cacheResponding();

    // since it is a normal request, attempt to send the packet
    bool success = memSidePorts[mem_side_port_id]->sendTimingReq(pkt);

    if (!success) {
        DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // restore the header delay as it is additive
        pkt->headerDelay = old_header_delay;

        // occupy until the header is sent
        reqLayers[mem_side_port_id]->failedTiming(src_port,
                                                  clockEdge(Cycles(1)));

        return false;
    }

    // remember where to route the response to
    if (expect_response) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = cpu_side_port_id;
    }

    reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

} // namespace gem5
