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

#include <cstdlib>
#include <iostream>

#include "DSENT.h"
#include "model/std_cells/StdCellLib.h"

using namespace std;

namespace DSENT
{
    static void performTimingOpt(const map<String, String> &params,
                                 Model *ms_model)
    {
        // Get the frequency it is optimizing to
        double freq = params.at("Frequency").toDouble();

        // Get all the starting net names
        const vector<String>& start_net_names =
            params.at("TimingOptimization->StartNetNames").split("[,]");

        ASSERT((start_net_names.size() > 0),
               "[Error] Expecting net names in TimingOptimization->StartNetNames");

        if(start_net_names[0] == "*")
        {
            // Optimize from all input ports
            ElectricalModel* electrical_model = (ElectricalModel*)ms_model;

            ElectricalTimingOptimizer timing_optimizer(
                    "Optimizer", electrical_model->getTechModel());
            timing_optimizer.setModel(electrical_model);
            timing_optimizer.construct();
            timing_optimizer.update();

            ElectricalTimingTree timing_tree(
                    timing_optimizer.getInstanceName(), &timing_optimizer);

            const Map<PortInfo*>* input_ports = timing_optimizer.getInputs();
            Map<PortInfo*>::ConstIterator it_begin = input_ports->begin();
            Map<PortInfo*>::ConstIterator it_end = input_ports->end();
            Map<PortInfo*>::ConstIterator it;
            for(it = it_begin; it != it_end; ++it)
            {
                const String& net_name = it->first;
                Log::printLine("Optimizing net: " + net_name);
                timing_tree.performTimingOpt(
                        timing_optimizer.getNet(net_name, makeNetIndex(0)), 1.0 / freq);
            }

            // Loop the second times 
            for(it = it_begin; it != it_end; ++it)
            {
                const String& net_name = it->first;
                Log::printLine("Optimizing net: " + net_name);
            }
        }
        else
        {
            // TODO : parse the net name so that we could do hierarchical optimization
            // Currently we can only optimize timing at the top level
            ElectricalModel* electrical_model = (ElectricalModel*)ms_model;
            ElectricalTimingTree timing_tree(
                    electrical_model->getInstanceName(), electrical_model);

            for(unsigned int i = 0; i < start_net_names.size(); ++i)
            {
                const String& net_name = start_net_names[i];
                timing_tree.performTimingOpt(
                        electrical_model->getNet(net_name), 1.0 / freq);
            }
        }
    }

    static void reportTiming(const map<String, String> &params, Model *ms_model)
    {
        // Get all the starting net names
        const vector<String>& start_net_names =
            params.at("ReportTiming->StartNetNames").split("[,]");

        ElectricalModel* electrical_model = (ElectricalModel*)ms_model;
        ElectricalTimingTree timing_tree(
                electrical_model->getInstanceName(), electrical_model);

        cout << "Report timing:" << endl;
        cout << "==============" << endl;
        for(unsigned int i = 0; i < start_net_names.size(); ++i)
        {
            const String& net_name = start_net_names[i];
            double timing = timing_tree.performCritPathExtract(electrical_model->getNet(net_name));
            cout << net_name << " = " << timing << endl;
        }
        cout << "==============" << endl;
    }

    static Model *buildModel(const map<String, String> &params,
                             TechModel *tech_model)
    {
        // Create the model specified
        const String& model_name = params.at("ModelName");
        Model *ms_model = ModelGen::createModel(model_name, model_name,
                                                tech_model);

        // Construct the model
        // Read all parameters the model requires
        const vector<String>* parameter_names = ms_model->getParameterNames();
        // For all parameters, grab values from the config file
        for(vector<String>::const_iterator it = parameter_names->begin();
            it != parameter_names->end(); ++it)
        {
            const String& parameter_name = *it;
            // If it exists in the config file, set the parameter
            if(params.count(parameter_name) > 0)
            {
                ms_model->setParameter(parameter_name,
                                       params.at(parameter_name));
            }
        }

        ms_model->construct();

        // Update the model
        // Read all properties the model requires
        const vector<String>* property_names = ms_model->getPropertyNames();
        // For all properties, grab values from the config file
        for(vector<String>::const_iterator it = property_names->begin();
            it != property_names->end(); ++it)
        {
            const String& property_name = *it;
            // If it exists in the config file, set the parameter
            if(params.count(property_name) > 0)
            {
                ms_model->setProperty(property_name,
                                      params.at(property_name));
            }
        }
        ms_model->update();

        // Evaluate the model
        // Perform timing optimization if needed
        if(params.find("IsPerformTimingOptimization") != params.end() &&
           params.at("IsPerformTimingOptimization").toBool())
        {
            performTimingOpt(params, ms_model);
        }
        ms_model->evaluate();

        // Report timing if needed
        if(params.count("IsReportTiming") > 0 &&
           params.at("IsReportTiming") != "false")
        {
            reportTiming(params, ms_model);
        }

        return ms_model;
    }

    static const void* processQuery(const String& query_str_,
                                    Model *ms_model, bool is_print_)
    {
        vector<String> type_split = query_str_.splitByString(Model::TYPE_SEPARATOR);
        ASSERT((type_split.size() == 2), "[Error] Invalid query format: " + query_str_);
        String query_type = type_split[0];

        vector<String> detail_split =
            type_split[1].splitByString(Model::DETAIL_SEPARATOR);

        ASSERT((detail_split.size() == 2), "[Error] Invalid query format: " + query_str_);
        String query_detail = detail_split[1];

        vector<String> subfield_split =
            detail_split[0].splitByString(Model::SUBFIELD_SEPARATOR);

        ASSERT(((subfield_split.size() == 2) || (subfield_split.size() == 1)),
               "[Error] Invalid query format: " + query_str_);

        String query_hier = subfield_split[0];
        String query_subfield = "";

        if(subfield_split.size() == 2)
        {
            query_subfield = subfield_split[1];
        }

        const void* query_result = ms_model->parseQuery(query_type, query_hier,
                                                        query_subfield);
        if(query_type == "Property")
        {
            const PropertyMap* property = (const PropertyMap*)query_result;
            if(is_print_)
            {
                cout << *property;
            }
        }
        else if(query_type == "Parameter")
        {
            const ParameterMap* parameter = (const ParameterMap*)query_result;
            if(is_print_)
            {
                cout << *parameter;
            }
        }
        else if(query_type.contain("Hier"))
        {
            const Model* model = (const Model*)query_result;
            if(is_print_)
            {
                model->printHierarchy(query_type, query_subfield, "", query_detail, cout);
            }
        }
        else
        {
            const Result* result = (const Result*)query_result;
            if(is_print_)
            {
                result->print(query_type + Model::TYPE_SEPARATOR + query_hier + 
                        Model::SUBFIELD_SEPARATOR + query_subfield, query_detail, cout);
            }
        }
        return query_result;
    }

    void processQuery(const vector<String> &queries,
                      Model *ms_model, vector<String> &outputs)
    {
        for(unsigned int i = 0; i < queries.size(); ++i)
        {
            const String& curr_query = queries[i];
            processQuery(curr_query, ms_model, true);

        }
    }

    static TechModel* constructTechModel(const map<String, String>& params)
    {
        // Allocate static TechModel instance
        const String& electrical_tech_model_filename =
            params.at("ElectricalTechModelFilename");

        TechModel* tech_model = new TechModel();
        tech_model->readFile(electrical_tech_model_filename);

        if (params.count("PhotonicTechModelFilename") != 0) {
            const String& photonic_tech_model_filename =
                params.at("PhotonicTechModelFilename");
            tech_model->readFile(photonic_tech_model_filename);
        }

        // Allocate static StdCellLib instance
        StdCellLib* std_cell_lib = new StdCellLib(tech_model);

        // Set the StdCellLib pointer in static TechModel instance
        tech_model->setStdCellLib(std_cell_lib);
        return tech_model;
    }

    Model *initialize(const char *config_file_name, map<String, String> &config)
    {
        // Init the log file
        Log::allocate("/tmp/dsent.log");

        // Init the config file
        LibUtil::readFile(config_file_name, config);

        // Overwrite the technology file
        TechModel *tech_model = constructTechModel(config);

        // Build the specified model in the config file
        return buildModel(config, tech_model);
    }

    void finalize(map<String, String> &config, Model *ms_model)
    {
        // Delete the model
        delete ms_model;

        // Discard all the (key, value) pairs.
        config.clear();

        // Release the log file
        Log::release();
    }

    void run(const map<String, String> &params, Model *ms_model,
             map<string, double> &outputs)
    {
        // Process the specified queries
        const auto &it = params.find("EvaluateString");
        if(it == params.end()) {
            return;
        }

        String eval_str = it->second;

        if (eval_str == "") {
            return;
        }

        DSENTCalculator calc;
        calc.evaluateString(eval_str, params, ms_model, outputs);
    }

    DSENTCalculator::DSENTCalculator() {}

    DSENTCalculator::~DSENTCalculator() {}

    double DSENTCalculator::getEnvVar(const String& var_name_,
                                      const map<String, String> &config,
                                      Model *ms_model) const
    {
        if (m_var_.keyExist(var_name_)) {
            return m_var_.get(var_name_);
        } else if (config.count(var_name_) > 0) {
            return config.at(var_name_);
        } else {
            // Wish there was a way to not have to pass in a stream if we aren't
            // doing anything with it
            const Result* result = (const Result*)DSENT::processQuery(
                var_name_ + "@0", ms_model, false);
            return result->calculateSum();
        }
    }
} // namespace DSENT
