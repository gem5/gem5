#include "bootcamp/inspector-gadget/inspector_gadget.hh"

#include <algorithm>
#include <cmath>

#include "debug/InspectorGadget.hh"

namespace gem5
{

InspectorGadget::InspectorGadget(const InspectorGadgetParams& params):
ClockedObject(params),
cpuSidePort(this, name()+".cpu_side_port"),
memSidePort(this, name()+".mem_side_port"),
inspectionBufferEntries(params.inspection_buffer_entries),
inspectionBuffer(clockPeriod()),
responseBufferEntries(params.response_buffer_entries),
responseBuffer(clockPeriod()),
nextReqSendEvent([this](){processNextReqSendEvent();}, name()+\
".nextReqSendEvent"),
nextReqRetryEvent([this](){processNextReqRetryEvent();}, name()+\
".nextReqRetryEvent"),
nextRespSendEvent([this](){processNextRespSendEvent();}, name()+\
".nextRespSendEvent"),
nextRespRetryEvent([this](){processNextRespRetryEvent();}, name()+\
".nextRespRetryEvent"),
stats(this)
{
}

void
InspectorGadget::init()
{
cpuSidePort.sendRangeChange();
}

Port&
InspectorGadget::getPort(const std::string& if_name, PortID idx)
{
// the filed idx will be useful for vector ports or having multiple peers.
if (if_name == "cpu_side_port"){
    return cpuSidePort;
}else if (if_name == "mem_side_port"){
    return memSidePort;
}else {
    return ClockedObject::getPort(if_name, idx);
}
}

Tick
InspectorGadget::align(Tick when)
{
// clockEdge(cycles_count) returns the aligned tick after
// advancement of cycles_count
return clockEdge((Cycles) std::ceil((when - curTick()) / clockPeriod()));
}

bool
InspectorGadget::MemSidePort::recvTimingResp(PacketPtr pkt)
{
DPRINTF(InspectorGadget, "%s: Recieved pkt: %s \
in timing mode.\n",__func__,pkt->print());
if (owner->recvTimingResp(pkt)) {
    return true;
}
needToSendRetry=true;
return false;
}

bool
InspectorGadget::recvTimingResp(PacketPtr pkt)
{
if (responseBuffer.size() >= responseBufferEntries) {
    return false;
}
responseBuffer.push(pkt, curTick());
//scheduleNextRespEvent(nextCycle());
scheduleNextRespSendEvent(nextCycle());
return true;
}

void
InspectorGadget::MemSidePort::recvReqRetry()
{
panic_if(!blocked(), "should never recieve retry if not blocked");
DPRINTF(InspectorGadget, "%s: Recieved retry signal.\n",__func__);
PacketPtr pkt=blockedPacket;
blockedPacket=nullptr;
sendPacket(pkt);

if ( ! blocked()) {
    owner->recvReqRetry();
}
}

void
InspectorGadget::recvReqRetry()
{
scheduleNextReqSendEvent(nextCycle());
}

bool
InspectorGadget::CPUSidePort::recvTimingReq(PacketPtr pkt)
{
DPRINTF(InspectorGadget, "%s: recieved pkt: %s in \
timing mode.\n",__func__,pkt->print());
if (owner->recvTimingReq(pkt)){
    return true;
}
needToSendRetry=true;
return false;
}

bool
InspectorGadget::recvTimingReq(PacketPtr pkt)
{
if (inspectionBuffer.size() >= inspectionBufferEntries){
    return false;
}
inspectionBuffer.push(pkt, curTick());
scheduleNextReqSendEvent(nextCycle());	// it will try to schedule
return true;
}

Tick
InspectorGadget::CPUSidePort::recvAtomic(PacketPtr pkt)
{
DPRINTF(InspectorGadget, "%s: Recieved pkt: %s in \
atomic mode.\n",__func__,pkt->print());
return owner->recvAtomic(pkt);
}

Tick
InspectorGadget::recvAtomic(PacketPtr pkt)
{
return clockPeriod() + memSidePort.sendAtomic(pkt);
}

void
InspectorGadget::CPUSidePort::recvFunctional(PacketPtr pkt)
{
DPRINTF(InspectorGadget, "%s: Recieved pkt: %s in \
functional mode.\n",__func__,pkt->print());
owner->recvFunctional(pkt);
}

void
InspectorGadget::recvFunctional(PacketPtr pkt)
{
memSidePort.sendFunctional(pkt);
}

AddrRangeList
InspectorGadget::CPUSidePort::getAddrRanges() const
{
return owner->getAddrRanges();
}

AddrRangeList
InspectorGadget::getAddrRanges() const
{
return memSidePort.getAddrRanges();
}

void
InspectorGadget::MemSidePort::sendPacket(PacketPtr pkt)
{
panic_if(blocked(), "Should never try to send if blocked");
DPRINTF(InspectorGadget, "%s: Sending Packet %s.\n",__func__,pkt->print());
if (!sendTimingReq(pkt)){
    DPRINTF(InspectorGadget, "%s: Failed to send pkt:\
%s.\n",__func__,pkt->print());
    blockedPacket=pkt;
}
}

void
InspectorGadget::CPUSidePort::sendPacket(PacketPtr pkt)
{
panic_if(blocked(), "Should never try if blocked");

DPRINTF(InspectorGadget, "%s Sending pkt: %s.\n",__func__,pkt->print());
if ( ! sendTimingResp(pkt)) {
    DPRINTF(InspectorGadget, \
"%s: Failed to send: %s.\n",__func__,pkt->print());
    blockedPacket=pkt;
}
}


void
InspectorGadget::CPUSidePort::recvRespRetry()
{
panic_if(!blocked(), "Should never recieve retry if not blocked");

DPRINTF(InspectorGadget, "%s: Recieved retry signal.\n",__func__);
PacketPtr pkt=blockedPacket;
blockedPacket=nullptr;
sendPacket(pkt);

if ( ! blocked()) {
    owner->recvRespRetry();
}
}

void
InspectorGadget::recvRespRetry()
{
scheduleNextRespSendEvent(nextCycle());
}

void
InspectorGadget::processNextReqSendEvent()
{
panic_if(memSidePort.blocked(),"Should never try to send if blocked");
panic_if(!inspectionBuffer.hasReady(curTick()), "Should never \
try to send if no ready packets");
stats.numRequestsFwded++;
stats.totalInspectionBufferLatency += curTick() - inspectionBuffer.frontTime();


PacketPtr pkt=inspectionBuffer.front();
memSidePort.sendPacket(pkt);
inspectionBuffer.pop();

scheduleNextReqRetryEvent(nextCycle());
scheduleNextReqSendEvent(nextCycle());
}

void
InspectorGadget::scheduleNextReqSendEvent(Tick when)
{
bool port_avail = !memSidePort.blocked();
bool have_items = !inspectionBuffer.empty();

if (port_avail && have_items && !nextReqSendEvent.scheduled()) {
Tick schedule_time = align(std::max(when,inspectionBuffer.firstReadyTime()));
    schedule(nextReqSendEvent, schedule_time);
}
}


void
InspectorGadget::processNextReqRetryEvent()
{
panic_if(!cpuSidePort.needRetry(), "Should never \
try to send retry if not needed");
cpuSidePort.sendRetryReq();
}

void
InspectorGadget::scheduleNextReqRetryEvent(Tick when)
{
if (cpuSidePort.needRetry() && !nextReqRetryEvent.scheduled()) {
    schedule(nextReqRetryEvent, align(when));
}
}

void
InspectorGadget::processNextRespSendEvent()
{
panic_if(cpuSidePort.blocked(), "Should never try to send if blocked");
panic_if(!responseBuffer.hasReady(curTick()), "Should never \
try if no packet is ready");

stats.numResponsesFwded++;
stats.totalResponseBufferLatency += curTick() - responseBuffer.frontTime();
PacketPtr pkt=responseBuffer.front();
cpuSidePort.sendPacket(pkt);
responseBuffer.pop();

scheduleNextRespRetryEvent(nextCycle());
scheduleNextRespSendEvent(nextCycle());
}

void
InspectorGadget::scheduleNextRespSendEvent(Tick when)
{
bool port_avail = !cpuSidePort.blocked();
bool have_items = !responseBuffer.empty();

if (port_avail && have_items && !nextRespSendEvent.scheduled()) {
    Tick schedule_time = align(std::max(when, \
responseBuffer.firstReadyTime()));
    schedule(nextRespSendEvent, schedule_time);
}
}

void
InspectorGadget::processNextRespRetryEvent()
{
panic_if(!memSidePort.needRetry(), "Should never \
try to send retry if not needed");
memSidePort.sendRetryResp();
}

void
InspectorGadget::scheduleNextRespRetryEvent(Tick when)
{
if (memSidePort.needRetry() && !nextRespRetryEvent.scheduled()) {
    schedule(nextRespRetryEvent, align(when));
}
}

InspectorGadget::InspectorGadgetStats::InspectorGadgetStats(\
InspectorGadget* inspector_gadget):
statistics::Group(inspector_gadget),
ADD_STAT(totalInspectionBufferLatency, statistics::units::Tick::get(), \
"Total inspection buffer latency"),
ADD_STAT(numRequestsFwded, statistics::units::Count::get(), \
"No of requests Forwaded"),
ADD_STAT(totalResponseBufferLatency, statistics::units::Tick::get(), \
"Total response buffer latency"),
ADD_STAT(numResponsesFwded, statistics::units::Count::get(), \
"No of responses Forwarded")
{}


} // end of namespace gem5
