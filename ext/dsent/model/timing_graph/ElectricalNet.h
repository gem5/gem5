#ifndef __DSENT_MODEL_ELECTRICAL_NET_H__
#define __DSENT_MODEL_ELECTRICAL_NET_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalLoad;

    class ElectricalNet : public ElectricalTimingNode
    {
        public:
            ElectricalNet(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalNet();

        public:
            // Set distributed res/cap
            void setDistributedRes(double distributed_res_);
            void setDistributedCap(double distributed_cap_);
            // Get distributed res/cap
            double getDistributedRes() const;
            double getDistributedCap() const;
            // Calculate wiring delay (or net delay)
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            // get total amount of downstream load capacitance 
            double getTotalDownstreamCap() const;

            virtual bool isNet() const;
            
        private:
            // Disable copy constructor
            ElectricalNet(const ElectricalNet& net_);

        private:
            // Name of this instance
            String m_instance_name_;
            // Distributed capacitance and resistance of the net
            double m_distributed_res_;            
            double m_distributed_cap_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_NET_H__

