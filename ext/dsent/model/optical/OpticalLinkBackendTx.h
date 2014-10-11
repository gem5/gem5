#ifndef __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDTX_H__
#define __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDTX_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class OpticalLinkBackendTx : public ElectricalModel
    {
        // An optical link backend tx contains everything needed for thermal
        // tuning of rings, bit-reshuffling (if necessary), and serialization (if necessary)
        public:
            OpticalLinkBackendTx(const String& instance_name_, const TechModel* tech_model_);
            virtual ~OpticalLinkBackendTx();

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
        
    }; // class OpticalLinkBackendTx
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_OPTICALLINKBACKENDTX_H__

