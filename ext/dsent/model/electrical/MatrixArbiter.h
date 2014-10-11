#ifndef __DSENT_MODEL_ELECTRICAL_MATRIX_ARBITER_H__
#define __DSENT_MODEL_ELECTRICAL_MATRIX_ARBITER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class MatrixArbiter : public ElectricalModel
    {
        public:
            MatrixArbiter(const String& instance_name_, const TechModel* tech_model_);
            virtual ~MatrixArbiter();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual MatrixArbiter* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class MatrixArbiter
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_MATRIX_ARBITER_H__

