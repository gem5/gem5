#ifndef __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDRX_H__
#define __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDRX_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class OpticalLinkBackendRx : public ElectricalModel
    {
        // An optical link backend rx contains everything needed for thermal
        // tuning of rings, bit-reshuffling (if necessary), and deserialization (if necessary)
        public:
            OpticalLinkBackendRx(const String& instance_name_, const TechModel* tech_model_);
            virtual ~OpticalLinkBackendRx();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void propagateTransitionInfo();
            
        private:
            // Calculate ring tuning power
            double getRingTuningPower();
            // Calculate the degree of bit re-order muxing (for the bit-reshuffler)
            unsigned int getBitReorderDegree();
        
    }; // class OpticalLinkBackendRx
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDRX_H__

