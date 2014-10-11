#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALLASER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALLASER_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalLaser : public OpticalNode
    {
        public:
            OpticalLaser(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_);
            ~OpticalLaser();

        public:
            void setEfficiency(double efficiency_);
            double getEfficiency() const;
        
        private:
            // Disable copy constructor
            OpticalLaser(const OpticalLaser& node_);            
        
        private:
            // Laser efficiency
            double m_efficiency_;
        
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALLASER_H__

