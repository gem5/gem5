#ifndef __DSENT_MODEL_OPTICAL_RINGFILTER_H__
#define __DSENT_MODEL_OPTICAL_RINGFILTER_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class RingFilter : public OpticalModel
    {
        public:
            RingFilter(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RingFilter();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();

    }; // class RingFilter
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_RINGFILTER_H__

