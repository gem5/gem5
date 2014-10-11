
#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalReceiver.h"

namespace DSENT
{
    OpticalDetector::OpticalDetector(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_)
        : OpticalNode(OpticalNode::DETECTOR, instance_name_, model_, wavelengths_), m_receiver_(receiver_), m_responsivity_(0)
    {
        m_sensitivity_ = 0.0;
    }

    OpticalDetector::~OpticalDetector()
    {

    }
    
    void OpticalDetector::setResponsivity(double responsivity_)
    {
        m_responsivity_ = responsivity_;
        return;
    }
    
    double OpticalDetector::getSensitivity(double ER_dB_) const
    {
        // Get responsivity (in Amps) of the receiver, divide by responsivity to get sensitivity in Watts
        return m_receiver_->getSensitivity(ER_dB_) / m_responsivity_;
    }
                
} // namespace DSENT


