#ifndef __DSENT_MODEL_STD_CELLS_ADDF_H__
#define __DSENT_MODEL_STD_CELLS_ADDF_H__

#include "util/CommonType.h"
#include "model/std_cells/StdCell.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    // A full adder standard cell
    class ADDF : public StdCell
    {
        public:
            ADDF(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ADDF();

        public:
            // Set a list of properties needed to update model
            void initProperties();
            // Cache the standard cell
            void cacheStdCell(StdCellLib* cell_lib_, double drive_strength_);
        private:
            TransitionInfo m_trans_P_;
            TransitionInfo m_trans_G_;
            TransitionInfo m_trans_CP_;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void evaluateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();
            
    }; // class ADDF
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_ADDF_H__

