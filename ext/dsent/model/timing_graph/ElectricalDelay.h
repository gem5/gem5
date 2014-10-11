#ifndef __DSENT_MODEL_ELECTRICAL_DELAY_H__
#define __DSENT_MODEL_ELECTRICAL_DELAY_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalLoad;

    class ElectricalDelay : public ElectricalTimingNode
    {
        public:
            ElectricalDelay(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalDelay();

        public:
            // Specify an ideal delay
            void setDelay(double delay_);
            // Get the ideal delay
            double getDelay() const;
            // Calculate delay
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            // get total amount of downstream load capacitance 
            double getTotalDownstreamCap() const;
            
        private:
            // Disable copy constructor
            ElectricalDelay(const ElectricalDelay& net_);

        private:
            // The amount of ideal delay
            double m_delay_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DELAY_H__

