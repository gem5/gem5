#ifndef __DSENT_MODEL_MODELGEN_H__
#define __DSENT_MODEL_MODELGEN_H__

#include "util/CommonType.h"

namespace DSENT
{
    class Model;
    class ElectricalModel;
    class StdCell;
    class TechModel;

    class ModelGen
    {
        public:
            // Create the model corresponding to the given String
            static Model* createModel(const String& model_name_, const String& instance_name_, const TechModel* tech_model_);
            // Create the standard cell corresponding to the given String
            static StdCell* createStdCell(const String& std_cell_name_, const String& instance_name_, const TechModel* tech_model_);
            // Create the ram corresponding to the given String
            static ElectricalModel* createRAM(const String& ram_name_, const String& instance_name_, const TechModel* tech_model_);
            // Create the crossbar corresponding to the given String
            static ElectricalModel* createCrossbar(const String& crossbar_name_, const String& instance_name_, const TechModel* tech_model_);
            // Print the available models
            static void printAvailableModels();
    };
} // namespace DSENT

#endif // __DSENT_MODEL_MODELGEN_H__

