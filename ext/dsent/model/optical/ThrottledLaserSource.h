#ifndef __DSENT_MODEL_OPTICAL_THROTTLEDLASERSOURCE_H__
#define __DSENT_MODEL_OPTICAL_THROTTLEDLASERSOURCE_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class OpticalWavelength;

    // A laser source that outputs some number of wavelengths. This laser
    // full on/off power gating, thus all power are event-based energies
    class ThrottledLaserSource : public OpticalModel
    {
        public:
            ThrottledLaserSource(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ThrottledLaserSource();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            void constructModel();
            void updateModel();
            void evaluateModel();
            void useModel();
            
        private:
            // Data structure containing the wavelengths that this laser outputs
            OpticalWavelength* m_wavelength_;
            
    }; // class ThrottledLaserSource
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_THROTTLEDLASERSOURCE_H__

