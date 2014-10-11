
#include "model/optical_graph/OpticalLaser.h"

namespace DSENT
{
    OpticalLaser::OpticalLaser(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_)
        : OpticalNode(OpticalNode::LASER, instance_name_, model_, wavelengths_), m_efficiency_(0)
    {
        
    }

    void OpticalLaser::setEfficiency(double efficiency_)
    {
        m_efficiency_ = efficiency_;
        return;
    }
    
    double OpticalLaser::getEfficiency() const
    {
        return m_efficiency_;
    }
    
    OpticalLaser::~OpticalLaser()
    {

    }

            
} // namespace DSENT


