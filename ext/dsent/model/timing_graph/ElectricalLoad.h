#ifndef __DSENT_MODEL_ELECTRICAL_LOAD_H__
#define __DSENT_MODEL_ELECTRICAL_LOAD_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalModel;
    class ElectricalDriver;
    
    class ElectricalLoad : public ElectricalTimingNode
    {
        public:
            ElectricalLoad(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalLoad();

        public:
            // Set the input capacitance of this input port
            void setLoadCap(double load_cap_);
            // Get the load capacitance
            double getLoadCap() const;
            // Get total load capacitance
            double getTotalDownstreamCap() const;
            // Calculate delay due to total load capacitance
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;

            bool isLoad() const;
            
        private:
            // Disable copy constructor
            ElectricalLoad(const ElectricalLoad& load_);

        private:
            // Load capacitance
            double m_load_cap_;
    };
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_LOAD_H__

