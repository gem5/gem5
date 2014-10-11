#ifndef __DSENT_MODEL_OPTICAL_SWSRLINK_H__
#define __DSENT_MODEL_OPTICAL_SWSRLINK_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class SWSRLink : public OpticalModel
    {
        // A SWSR Link consists of a laser, a modulator (the writer) and a variable
        // number of readers
        public:
            SWSRLink(const String& instance_name_, const TechModel* tech_model_);
            virtual ~SWSRLink();

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
            void buildDetector();

    }; // class SWSRLink
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_SWSRLINK_H__

