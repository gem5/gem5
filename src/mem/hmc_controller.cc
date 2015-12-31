#include "base/random.hh"
#include "debug/HMCController.hh"
#include "mem/hmc_controller.hh"

HMCController::HMCController(const HMCControllerParams* p) :
    NoncoherentXBar(p),
    n_master_ports(p->port_master_connection_count),
    rr_counter(0)
{
    assert(p->port_slave_connection_count == 1);
}

HMCController*
HMCControllerParams::create()
{
    return new HMCController(this);
}

// Since this module is a load distributor, all its master ports have the same
//  range so we should keep only one of the ranges and ignore the others
void HMCController::recvRangeChange(PortID master_port_id)
{
    if (master_port_id == 0)
    {
       gotAllAddrRanges = true;
       BaseXBar::recvRangeChange(master_port_id);
    }
    else
        gotAddrRanges[master_port_id] = true;
}

int HMCController::rotate_counter()
{
    int current_value = rr_counter;
    rr_counter++;
    if (rr_counter == n_master_ports)
        rr_counter = 0;
    return current_value;
}

bool HMCController::recvTimingReq(PacketPtr pkt, PortID slave_port_id)
{
    // determine the source port based on the id
    SlavePort *src_port = slavePorts[slave_port_id];

    // we should never see express snoops on a non-coherent component
    assert(!pkt->isExpressSnoop());

    // For now, this is a simple round robin counter, for distribution the
    //  load among the serial links
    PortID master_port_id = rotate_counter();

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[master_port_id]->tryTiming(src_port)) {
        DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

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
    const bool expect_response = pkt->needsResponse() &&
        !pkt->cacheResponding();

    // since it is a normal request, attempt to send the packet
    bool success = masterPorts[master_port_id]->sendTimingReq(pkt);

    if (!success)  {
        DPRINTF(HMCController, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // restore the header delay as it is additive
        pkt->headerDelay = old_header_delay;

        // occupy until the header is sent
        reqLayers[master_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));

        return false;
    }

    // remember where to route the response to
    if (expect_response) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = slave_port_id;
    }

    reqLayers[master_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[slave_port_id][master_port_id]++;
    pktSize[slave_port_id][master_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}
