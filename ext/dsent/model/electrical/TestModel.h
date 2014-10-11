#ifndef __DSENT_MODEL_ELECTRICAL_TESTMODEL_H__
#define __DSENT_MODEL_ELECTRICAL_TESTMODEL_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class TestModel : public ElectricalModel
    {
        public:
            TestModel(const String& instance_name_, const TechModel* tech_model_);
            virtual ~TestModel();

        public:
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual TestModel* clone() const;

        protected:
            // Build the model
            void constructModel();
            void updateModel();
            void evaluateModel();

    }; // class TestModel
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_TESTMODEL_H__

