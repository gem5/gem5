#ifndef __DSENT_MODEL_ELECTRICAL_DRIVER_MULTIPLIER_H__
#define __DSENT_MODEL_ELECTRICAL_DRIVER_MULTIPLIER_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    // Simple class that can be used to mimic the presence of multiple drivers
    // output drivers (each driving one of the downstream loads) when only one
    // such driver has been instantiated (such as models that take advantage of
    // bit duplicattion). When the downsream loads differ in load cap, it
    // just returns the largest of the caps
    class ElectricalDriverMultiplier : public ElectricalTimingNode
    {
        public:
            ElectricalDriverMultiplier(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalDriverMultiplier();

        public:
            // Calculate drive resistance of this node;
            double calculateDriveRes(double input_drive_res_) const;
            // Calculate wiring delay (or net delay)
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get total amount of downstream load capacitance 
            double getTotalDownstreamCap() const;
            
        private:
            // Disable copy constructor
            ElectricalDriverMultiplier(const ElectricalDriverMultiplier& net_);

        private:
            // Name of this instance
            String m_instance_name_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DRIVER_MULTIPLIER_H__

