#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalFilter : public OpticalNode
    {
        public:
            OpticalFilter(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool drop_all_, const WavelengthGroup& drop_wavelengths_);
            ~OpticalFilter();

        public:
            // Get the drop all flag
            bool getDropAll() const;
            // Get drop wavelengths
            WavelengthGroup getDropWavelengths() const;
            // Set and get the drop loss
            void setDropLoss(double drop_loss_);
            double getDropLoss() const;
            // Set and get drop port
            void setDropPort(OpticalNode* drop_port_);
            OpticalNode* getDropPort();
            // Checks to see if a set of wavelengths will be dropped
            bool isDropped(const WavelengthGroup& wavelengths_) const;
            
        private:
            // Disable copy constructor
            OpticalFilter(const OpticalFilter& node_);

        private:
            // Whether to drop all the optical signal for the drop wavelengths
            // i.e. so that the drop wavelengths are not traced anymore
            const bool m_drop_all_;
            // The loss incurred from in to drop port
            double m_drop_loss_;
            // The wavelengths that are dropped
            const WavelengthGroup m_drop_wavelengths_;
            // The node at the drop port
            OpticalNode* m_drop_port_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__

