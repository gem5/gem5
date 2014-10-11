#ifndef __DSENT_MODEL_STD_CELLS_INV_H__
#define __DSENT_MODEL_STD_CELLS_INV_H__

#include "util/CommonType.h"
#include "model/std_cells/StdCell.h"

namespace DSENT
{
    class INV : public StdCell
    {
        public:
            INV(const String& instance_name_, const TechModel* tech_model_);
            virtual ~INV();

        public:
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Cache the standard cell
            void cacheStdCell(StdCellLib* cell_lib_, double drive_strength_);      

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void evaluateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

    }; // class INV
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_INV_H__

