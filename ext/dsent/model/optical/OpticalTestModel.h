#ifndef __DSENT_MODEL_OPTICAL_OPTICALTESTMODEL_H__
#define __DSENT_MODEL_OPTICAL_OPTICALTESTMODEL_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class OpticalTestModel : public OpticalModel
    {
        public:
            OpticalTestModel(const String& instance_name_, const TechModel* tech_model_);
            virtual ~OpticalTestModel();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();

    }; // class OpticalTestModel
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_RINGLASERSOURCE_H__

