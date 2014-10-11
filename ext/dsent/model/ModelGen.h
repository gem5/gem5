/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

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

