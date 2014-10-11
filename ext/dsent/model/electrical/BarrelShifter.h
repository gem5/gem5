#ifndef __DSENT_MODEL_ELECTRICAL_BARRELSHIFTER_H__
#define __DSENT_MODEL_ELECTRICAL_BARRELSHIFTER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // A model on N-bit barrel-shifter. Shifts to the right by X
    class BarrelShifter : public ElectricalModel
    {
        public:
            BarrelShifter(const String& instance_name_, const TechModel* tech_model_);
            virtual ~BarrelShifter();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual BarrelShifter* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class BarrelShifter
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_H__

