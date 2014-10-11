#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalReceiver;

    class OpticalDetector : public OpticalNode
    {
        public:
            OpticalDetector(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_);
            ~OpticalDetector();

        public:
            // Set the responsitivity of the photodetector
            void setResponsivity(double responsivity_);
            
            // Get the detector sensitivity given an extinction ratio (in Watts)
            double getSensitivity(double ER_dB_) const;
            
            // Ask the receiver for its power (ONLY use for power optimization, as this
            // assumes an activity of 1.0)
            double getPower() const;
        
        private:
            // Disable copy constructor
            OpticalDetector(const OpticalDetector& node_);            
        
        private:
            // The required laser power
            double m_sensitivity_;
            // The receiver connected to this detector
            OpticalReceiver* m_receiver_;
            // Responsivity of the photodetector
            double m_responsivity_;
        
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__

