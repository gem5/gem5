#ifndef __DSENT_MODEL_ELECTRICAL_DRIVER_H__
#define __DSENT_MODEL_ELECTRICAL_DRIVER_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalModel;

    class ElectricalDriver : public ElectricalTimingNode
    {
        public:
            ElectricalDriver(const String& instance_name_, ElectricalModel* model_, bool sizable_);
            virtual ~ElectricalDriver();

        public:
            // Set the output resistance of this driver
            void setOutputRes(double output_res_);            
            // Get the output resistance of this driver
            double getOutputRes() const;
            // Calculate delay due to total load capacitance
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            
            // Get whether the driver is sizable
            bool isSizable() const;
            // Return true if the instance has minimum driving strength
            bool hasMinDrivingStrength() const;
            // Return true if the instance has maximum driving strength
            bool hasMaxDrivingStrength() const;
            // Increase driving strength index by 1
            void increaseDrivingStrength();
            // Decrease driving strength index by 1
            void decreaseDrivingStrength();
                        
            bool isDriver() const;                        
                        
        private:
            // Disable copy constructor
            ElectricalDriver(const ElectricalDriver& port_);

        private:
            // Name of this instance
            String m_instance_name_;
            // Output resistance
            double m_output_res_;
            // Sizable flag
            bool m_sizable_;
    };
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DRIVER_H__

