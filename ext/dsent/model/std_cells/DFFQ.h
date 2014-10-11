#ifndef __DSENT_MODEL_STD_CELLS_DFFQ_H__
#define __DSENT_MODEL_STD_CELLS_DFFQ_H__

#include "util/CommonType.h"
#include "model/std_cells/StdCell.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    class DFFQ : public StdCell
    {
        // A DQ flip-flop
        public:
            DFFQ(const String& instance_name_, const TechModel* tech_model_);
            virtual ~DFFQ();

        public:
            // Set a list of properties' name needed to construct model
            void initProperties();
            // Cache the standard cell
            void cacheStdCell(StdCellLib* cell_lib_, double drive_strength_);
            
        private:
            TransitionInfo m_trans_M_;

        protected:
            // Build the model            
            virtual void constructModel();
            virtual void updateModel();
            virtual void evaluateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

    }; // class DFFQ
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_INV_H__

