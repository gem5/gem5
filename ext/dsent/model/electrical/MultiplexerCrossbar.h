#ifndef __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_CROSSBAR_H__
#define __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_CROSSBAR_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // A model for a NxM W-bit multiplexer-based crossbar
    class MultiplexerCrossbar : public ElectricalModel
    {
        public:
            MultiplexerCrossbar(const String& instance_name_, const TechModel* tech_model_);
            virtual ~MultiplexerCrossbar();

        public:
            // Set a list of paramerters' name needed to construct model 
            void initParameters();
            // Set a list of peroperties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual MultiplexerCrossbar* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void propagateTransitionInfo();

        private:
            // Disable copy constructor
            MultiplexerCrossbar(const MultiplexerCrossbar& crossbar_);
    }; // class MultiplexerCrossbar
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_CROSSBAR_H__

