#ifndef __DSENT_MODEL_OPTICAL_SWMRLINK_H__
#define __DSENT_MODEL_OPTICAL_SWMRLINK_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class SWMRLink : public OpticalModel
    {
        // A SWMR Link consists of a laser, a modulator (the writer) and a variable
        // number of readers
        public:
            SWMRLink(const String& instance_name_, const TechModel* tech_model_);
            virtual ~SWMRLink();

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
            void buildLaser();
            void buildModulator();
            void buildDetectors();

    }; // class SWMRLink
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_SWMRLINK_H__

