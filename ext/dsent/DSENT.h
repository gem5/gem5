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

#ifndef __DSENT_DSENT_H__
#define __DSENT_DSENT_H__

#include <map>
#include <string>

// For DSENT operations
#include "libutil/Calculator.h"
#include "util/CommonType.h"
#include "util/Result.h"
#include "model/Model.h"
#include "model/ModelGen.h"

// For timing optimization
#include "model/ElectricalModel.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalTimingTree.h"
#include "model/timing_graph/ElectricalTimingOptimizer.h"
#include "model/PortInfo.h"

namespace DSENT
{
    using LibUtil::Calculator;

    class DSENTCalculator : public Calculator
    {
        public:
            DSENTCalculator();
            virtual ~DSENTCalculator();

        protected:
            virtual double getEnvVar(
                const String& var_name_,
                const std::map<String, String> &Config,
                Model *ms_model) const;
    };

    Model *initialize(const char *config_file_name,
                      std::map<String, String> &config);

    void finalize(std::map<String, String> &config,
                  Model *ms_model);

    void run(const std::map<String, String> &config, Model *ms_model,
             std::map<std::string, double> &outputs);
} // namespace DSENT

#endif // __DSENT_DSENT_H__
