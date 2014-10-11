#include "DSENT.h"

#include <cstdlib>
#include <iostream>

namespace DSENT
{
    Model* DSENT::ms_model_ = NULL;
    bool DSENT::ms_is_verbose_ = false;

    void DSENT::run(int argc_, char** argv_)
    {
        // Initialize DSENT framework (setup log file, config file, ...)
        initialize(argc_, argv_);

        // Build the specified model in the config file
        buildModel();

        // Process the specified queries
        processQuery();
        // Process the specified evaluation
        processEvaluate();

        // Finalize DSENT framework (close log file, ...)
        finalize();
        return;
    }

    void DSENT::setRuntimeOptions(OptionParser* option_parser_)
    {
        option_parser_->addOption("-cfg", "ConfigFilename", true, "filename", false, "",
                "Specify the config filename.");

        option_parser_->addOption("-available_models", "IsListModels", false, "", true, "false",
                "List available DSENT models.");

        option_parser_->addOption("-log", "LogFilename", true, "filename", true, "./dsent.log", 
                "Specify the log filename.");

        option_parser_->addOption("-overwrite", "OverwriteString", true, "options", true, "",
                "Overwrite dynamically the options set in the config file. Options are separated by a comma (;).");

        option_parser_->addOption("-overwrite_tech", "OverwriteTechString", true, "options", true, "",
                "Overwrite dynamically the options set in the technology file. Options are separated by a comma (;).");

        option_parser_->addOption("-print_config", "IsPrintConfig", false, "", true, "false", 
                "Print the config used at DSENT runtime.");

        option_parser_->addOption("-query", "QueryString", true, "query string", true, "",
                "Specify the list of items to query. This command is the same as owerwriting the 'QueryString'.");

        option_parser_->addOption("-eval", "EvaluateString", true, "evaluate string", true, "",
                "Specify the list of statements to evaluate. This command is the same as owerwriting the 'EvaluateString'.");

        option_parser_->addOption("-verbose", "IsVerbose", false, "", true, "false", 
                "Enable verbose mode which prints out more detailed messages.");
        return;
    }

    void DSENT::initialize(int argc_, char** argv_)
    {
        OptionParser* option_parser = new OptionParser();

        // Init the option parser and setup available options
        setRuntimeOptions(option_parser);

        // Parse the options
        option_parser->parseArguments(argc_, argv_);

        // If -available_models is specified, print out a list of available 
        // models and exit DSENT.
        if(option_parser->get("IsListModels").toBool())
        {
            ModelGen::printAvailableModels();
            exit(0);
        }

        // Init the log file
        Log::allocate(option_parser->get("LogFilename"));

        // Init the config file
        Config::allocate(option_parser->get("ConfigFilename"));
        Config* dsent_config = Config::getSingleton();

        // Overwrite the existing options
        dsent_config->readString(option_parser->get("OverwriteString"));

        // Overwrite the technology file
        dsent_config->constructTechModel(option_parser->get("OverwriteTechString"));

        ms_is_verbose_ = option_parser->get("IsVerbose").toBool();

        // Overwrite the query string if it is specified from command line
        if(option_parser->get("QueryString").size() != 0)
        {
            dsent_config->set("QueryString", option_parser->get("QueryString"));
        }
        // Overwrite the evaluation string if it is specified from command line
        if(option_parser->get("EvaluateString").size() != 0)
        {
            dsent_config->set("EvaluateString", option_parser->get("EvaluateString"));
        }

        // Print the config used for this run
        if(option_parser->get("IsPrintConfig").toBool())
        {
            if(ms_is_verbose_)
            {
                cout << "Configuration:" << endl;
                cout << "==============" << endl;
            }
            cout << *dsent_config;

            if(ms_is_verbose_)
            {
                cout << "==============" << endl;
            }
        }

        delete option_parser;
        return;
    }

    void DSENT::buildModel()
    {
        Config* dsent_config = Config::getSingleton();

        // Create the model specified
        const String& model_name = dsent_config->get("ModelName");
        ms_model_ = ModelGen::createModel(model_name, model_name, dsent_config->getTechModel());

        // Construct the model
        // Read all parameters the model requires
        const vector<String>* parameter_names = ms_model_->getParameterNames();
        // For all parameters, grab values from the config file
        for(vector<String>::const_iterator it = parameter_names->begin(); it != parameter_names->end(); ++it)
        {
            const String& parameter_name = *it;
            // If it exists in the config file, set the parameter
            if(dsent_config->keyExist(parameter_name))
            {
                ms_model_->setParameter(parameter_name, dsent_config->get(parameter_name));
            }
        }
        ms_model_->construct();

        // Update the model
        // Read all properties the model requires
        const vector<String>* property_names = ms_model_->getPropertyNames();
        // For all properties, grab values from the config file
        for(vector<String>::const_iterator it = property_names->begin(); it != property_names->end(); ++it)
        {
            const String& property_name = *it;
            // If it exists in the config file, set the parameter
            if(dsent_config->keyExist(property_name))
            {
                ms_model_->setProperty(property_name, dsent_config->get(property_name));
            }
        }
        ms_model_->update();

        // Evaluate the model
        // Perform timing optimization if needed
        if(dsent_config->getIfKeyExist("IsPerformTimingOptimization", "false").toBool())
        {
            performTimingOpt();
        }
        ms_model_->evaluate();

        // Report timing if needed
        if(dsent_config->getIfKeyExist("IsReportTiming", "false").toBool())
        {
            reportTiming();
        }

        return;
    }

    void DSENT::processQuery()
    {
        Config* dsent_config = Config::getSingleton();
        vector<String> queries = dsent_config->get("QueryString").split(" ;\r\n");

        if(ms_is_verbose_)
        {
            cout << "Query results:" << endl;
            cout << "==============" << endl;
        }

        for(unsigned int i = 0; i < queries.size(); ++i)
        {
            const String& curr_query = queries[i];

            if(ms_is_verbose_)
            {
                String str = "Process query: '" + curr_query + "'";
                cout << str << endl;
                cout << String(str.size(), '-') << endl;
            }

            processQuery(curr_query, true);

            if(ms_is_verbose_)
            {
                cout << endl;
            }
        }
        if(ms_is_verbose_)
        {
            cout << "==============" << endl;
        }
        return;
    }

    const void* DSENT::processQuery(const String& query_str_, bool is_print_)
    {
        vector<String> type_split = query_str_.splitByString(Model::TYPE_SEPARATOR);
        ASSERT((type_split.size() == 2), "[Error] Invalid query format: " + query_str_);
        String query_type = type_split[0];

        vector<String> detail_split = type_split[1].splitByString(Model::DETAIL_SEPARATOR);
        ASSERT((detail_split.size() == 2), "[Error] Invalid query format: " + query_str_);
        String query_detail = detail_split[1];

        vector<String> subfield_split = detail_split[0].splitByString(Model::SUBFIELD_SEPARATOR);
        ASSERT(((subfield_split.size() == 2) || (subfield_split.size() == 1)), "[Error] Invalid query format: " + query_str_);
        String query_hier = subfield_split[0];
        String query_subfield = "";
        if(subfield_split.size() == 2)
        {
            query_subfield = subfield_split[1];
        }

        const void* query_result = ms_model_->parseQuery(query_type, query_hier, query_subfield);
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

    void DSENT::finalize()
    {
        // Release the constructed model
        delete ms_model_;
        ms_model_ = NULL;

        // Release the config file
        Config::release();

        // Release the log file
        Log::release();

        return;
    }

    void DSENT::performTimingOpt()
    {
        Config* dsent_config = Config::getSingleton();

        // Get the frequency it is optimizing to
        double freq = dsent_config->get("Frequency").toDouble();

        // Get all the starting net names
        const vector<String>& start_net_names = dsent_config->get("TimingOptimization->StartNetNames").split("[,]");

        ASSERT((start_net_names.size() > 0), "[Error] Expecting net names in TimingOptimization->StartNetNames");

        if(start_net_names[0] == "*")
        {
            // Optimize from all input ports
            ElectricalModel* electrical_model = (ElectricalModel*)ms_model_;

            ElectricalTimingOptimizer timing_optimizer("Optimizer", electrical_model->getTechModel());
            timing_optimizer.setModel(electrical_model);
            timing_optimizer.construct();
            timing_optimizer.update();

            ElectricalTimingTree timing_tree(timing_optimizer.getInstanceName(), &timing_optimizer);

            const Map<PortInfo*>* input_ports = timing_optimizer.getInputs();
            Map<PortInfo*>::ConstIterator it_begin = input_ports->begin();
            Map<PortInfo*>::ConstIterator it_end = input_ports->end();
            Map<PortInfo*>::ConstIterator it;
            for(it = it_begin; it != it_end; ++it)
            {
                const String& net_name = it->first;
                Log::printLine("Optimizing net: " + net_name);
                timing_tree.performTimingOpt(timing_optimizer.getNet(net_name, makeNetIndex(0)), 1.0 / freq);
                //timing_tree.performTimingOpt(electrical_model->getNet(net_name, makeNetIndex(0)), 1.0 / freq);
            }
            // Loop the second times 
            for(it = it_begin; it != it_end; ++it)
            {
                const String& net_name = it->first;
                Log::printLine("Optimizing net: " + net_name);
                //timing_tree.performTimingOpt(timing_optimizer.getNet(net_name, makeNetIndex(0)), 1.0 / freq);
            }
        }
        else
        {
            // TODO : parse the net name so that we could do hierarchical optimization
            // Currently we can only optimize timing at the top level
            ElectricalModel* electrical_model = (ElectricalModel*)ms_model_;
            ElectricalTimingTree timing_tree(electrical_model->getInstanceName(), electrical_model);
            for(unsigned int i = 0; i < start_net_names.size(); ++i)
            {
                const String& net_name = start_net_names[i];
                timing_tree.performTimingOpt(electrical_model->getNet(net_name), 1.0 / freq);
            }
        }
        return;
    }

    void DSENT::reportTiming()
    {
        Config* dsent_config = Config::getSingleton();

        // Get all the starting net names
        const vector<String>& start_net_names = dsent_config->get("ReportTiming->StartNetNames").split("[,]");

        ElectricalModel* electrical_model = (ElectricalModel*)ms_model_;
        ElectricalTimingTree timing_tree(electrical_model->getInstanceName(), electrical_model);

        cout << "Report timing:" << endl;
        cout << "==============" << endl;
        for(unsigned int i = 0; i < start_net_names.size(); ++i)
        {
            const String& net_name = start_net_names[i];
            double timing = timing_tree.performCritPathExtract(electrical_model->getNet(net_name));
            cout << net_name << " = " << timing << endl;
        }
        cout << "==============" << endl;
        return;
    }

    void DSENT::processEvaluate()
    {
        Config* dsent_config = Config::getSingleton();

        // Return if EvaluatString is empty or not exists
        if(!dsent_config->keyExist("EvaluateString")) return;

        String eval_str = dsent_config->get("EvaluateString");

        if(eval_str == "") return;

        if(ms_is_verbose_)
        {
            cout << "Eval results:" << endl;
            cout << "==============" << endl;
        }

        //if(ms_is_verbose_)
        //{
        //    String str = "Process evaluation: '" + eval_str + "'";
        //    cout << str << endl;
        //    cout << String(str.size(), '-') << endl;
        //}
        DSENTCalculator calc;
        calc.evaluateString(eval_str);

        if(ms_is_verbose_)
        {
            cout << "==============" << endl;
        }
        return;
        return;
    }

    DSENT::DSENTCalculator::DSENTCalculator()
    {}

    DSENT::DSENTCalculator::~DSENTCalculator()
    {}

    double DSENT::DSENTCalculator::getEnvVar(const String& var_name_) const
    {
        if(m_var_.keyExist(var_name_))
        {
            return m_var_.get(var_name_);
        }
        else if(Config::getSingleton()->keyExist(var_name_))
        {
            return Config::getSingleton()->get(var_name_);
        }
        else
        {
            const Result* result = (const Result*)DSENT::processQuery(var_name_ + "@0", false);
            return result->calculateSum();
        }
    }
} // namespace DSENT

