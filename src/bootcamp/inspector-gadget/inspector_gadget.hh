#ifndef __BOOTCAMP_INSPECTOR_GADGET_INSPECTOR_GADGET_HH__
#define __BOOTCAMP_INSPECTOR_GADGET_INSPECTOR_GADGET_HH__

#include <queue>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/InspectorGadget.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class InspectorGadget: public ClockedObject
{
    private:
        EventFunctionWrapper nextReqSendEvent;
        void processNextReqSendEvent(void);
        void scheduleNextReqSendEvent(Tick when);
        // it will try to schedule


        EventFunctionWrapper nextReqRetryEvent;
        void processNextReqRetryEvent();
        void scheduleNextReqRetryEvent(Tick when);

        EventFunctionWrapper nextRespSendEvent;
        void processNextRespSendEvent();
        void scheduleNextRespSendEvent(Tick when);

        EventFunctionWrapper nextRespRetryEvent;
        void processNextRespRetryEvent();
        void scheduleNextRespRetryEvent(Tick when);

        // this functin will be useful for different
        // clock domain object communication
        Tick align(Tick when);
        //RequestPort mem_side_port;
        //ResponsePort cpu_side_port;

    private:
        class CPUSidePort: public ResponsePort
        {
            private:
                InspectorGadget* owner;
                // Flag to chceck whether we
                // rejected any request made by requestor
                bool needToSendRetry;
                // response packet blocked at
                // our end as requestor refused to accept
                PacketPtr blockedPacket;
            public:
                CPUSidePort(InspectorGadget* owner, \
                        const std::string& name):
                    ResponsePort(name), owner(owner), \
                    needToSendRetry(false), blockedPacket(nullptr)
                {}
                bool needRetry() const {return needToSendRetry;}
                bool blocked() const {return blockedPacket != nullptr;}
                void sendPacket(PacketPtr pkt);

                virtual AddrRangeList getAddrRanges() const override;
                virtual bool recvTimingReq(PacketPtr pkt) override;
                virtual Tick recvAtomic(PacketPtr pkt) override;
                virtual void recvFunctional(PacketPtr pkt) override;
                virtual void recvRespRetry() override;
        };

        class MemSidePort: public RequestPort
        {
            private:
                InspectorGadget* owner;
                // flag if we rejected or do not accept the response
                bool needToSendRetry;
                // if our request packet is not accepted by other peer
                PacketPtr blockedPacket;
            public:
                MemSidePort(InspectorGadget* owner, const std::string& name):
                    RequestPort(name), owner(owner), \
                    needToSendRetry(false), blockedPacket(nullptr)
                {}
                bool needRetry() const {return needToSendRetry;}
                bool blocked() const {return blockedPacket!=nullptr;}
                void sendPacket(PacketPtr pkt);

                virtual bool recvTimingResp(PacketPtr pkt) override;
                //virtual bool recvReqRetry(PacketPtr pkt) override;
                virtual void recvReqRetry() override;
        };
    private:
        template<typename T>
        class TimedQueue
        {
            private:
                Tick latency;
                std::queue<T> items;
                std::queue<Tick> insertionTimes;
            public:
                TimedQueue(Tick latency): latency(latency) {}
                void push(T item, Tick insertion_time) {
                    items.push(item);
                    insertionTimes.push(insertion_time);
                }
                void pop(){
                    items.pop();
                    insertionTimes.pop();
                }

                T& front() {return items.front();}
                Tick frontTime() { return insertionTimes.front();}
                bool empty() const { return items.empty();}
                size_t size() const { return items.size();}
                bool hasReady(Tick current_time) const {
                    if (empty()) {
                        return false;
                    }
                    return (current_time - insertionTimes.front()) >= latency;
                }
                Tick firstReadyTime() {
                    return insertionTimes.front() + latency;}
        };

    private:
        struct InspectorGadgetStats: public statistics::Group
            {
                statistics::Scalar totalInspectionBufferLatency;
                statistics::Scalar numRequestsFwded;
                statistics::Scalar totalResponseBufferLatency;
                statistics::Scalar numResponsesFwded;
                InspectorGadgetStats(InspectorGadget* inspector_gadget);
            };
    public:
        InspectorGadget(const InspectorGadgetParams& params);
        virtual void init() override;
        // this function is called by gem5 during m5.instantiate() to connect
        // the SImobjects using ports
        virtual Port& getPort(const std::string& if_name, \
            PortID idx=InvalidPortID) override;

        AddrRangeList getAddrRanges() const;
        bool recvTimingReq(PacketPtr pkt);
        Tick recvAtomic(PacketPtr pkt);
        void recvFunctional(PacketPtr pkt);
        void recvReqRetry();

        bool recvTimingResp(PacketPtr pkt);
        void recvRespRetry();
    private:
        CPUSidePort cpuSidePort;
        MemSidePort memSidePort;

        int inspectionBufferEntries;
        TimedQueue<PacketPtr> inspectionBuffer;
        int responseBufferEntries;
        TimedQueue<PacketPtr> responseBuffer;

        InspectorGadgetStats stats;

};
}	// end of namespace gem5
#endif	// header definition ends
