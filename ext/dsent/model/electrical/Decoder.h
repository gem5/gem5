#ifndef __DSENT_MODEL_ELECTRICAL_DECODER_H__
#define __DSENT_MODEL_ELECTRICAL_DECODER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class Decoder : public ElectricalModel
    {
        public:
            Decoder(const String& instance_name_, const TechModel* tech_model_);
            virtual ~Decoder();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual Decoder* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class Decoder
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DECODER_H__

