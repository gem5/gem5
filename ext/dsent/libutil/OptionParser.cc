#include "OptionParser.h"

#include <cstdlib>
#include <iostream>

namespace LibUtil
{
    using std::cout;
    using std::cerr;
    using std::endl;

    OptionParser::OptionInfo::OptionInfo(
            const String& var_name_,
            bool has_arg_, 
            const String& arg_name_, 
            bool has_default_arg_value_,
            const String& default_arg_value_, 
            const String& description_
            )
        : m_var_name_(var_name_),
        m_has_arg_(has_arg_), 
        m_arg_name_(arg_name_), 
        m_has_default_arg_value_(has_default_arg_value_),
        m_default_arg_value_(default_arg_value_),
        m_description_(description_)
    {}

    OptionParser::OptionInfo::~OptionInfo()
    {
    }

    OptionParser::OptionParser()
    {}

    OptionParser::~OptionParser()
    {
        clearPtrMap(&m_option_infos_);
    }

    void OptionParser::addOption(
            const String& option_name_, 
            const String& var_name_, 
            bool has_arg_, 
            const String& arg_name_, 
            bool has_default_arg_value_,
            const String& default_arg_value_, 
            const String& description_)
    {
        OptionInfo* option_info = new OptionInfo(var_name_, has_arg_, arg_name_, 
                has_default_arg_value_, default_arg_value_, description_);

        ASSERT(!m_option_infos_.keyExist(option_name_), "Option exists: " + option_name_);

        // Add the option name to an array for sorting
        m_option_names_.push_back(option_name_);

        // Add option info 
        m_option_infos_.set(option_name_, option_info);

        // Set the default argument value
        if(has_default_arg_value_)
        {
            set(var_name_, default_arg_value_);
        }

        return;
    }

    void OptionParser::parseArguments(int argc_, char** argv_)
    {
        bool is_print_options = false;
        int arg_idx = 0;

        while(arg_idx < argc_)
        {
            String option_name = String(argv_[arg_idx]);

            // Print the options page if -help is specified
            if(option_name == "-help")
            {
                is_print_options = true;
                break;
            }
            else if(m_option_infos_.keyExist(option_name))
            {
                const OptionInfo* option_info = m_option_infos_.get(option_name);
                const String& var_name = option_info->getVarName();
                if(option_info->hasArg())
                {
                    if((arg_idx + 1) >= argc_)
                    {
                        cerr << "[Error] Missing argument for option: '" << option_name << "'" << endl;
                        is_print_options = true;
                        break;
                    }

                    String option_arg = String(argv_[arg_idx + 1]);
                    set(var_name, option_arg);

                    arg_idx += 2;
                }
                else
                {
                    // If the option does not require an argument
                    // then set it to true
                    set(var_name, "true");

                    arg_idx += 1;
                }
            }
            else
            {
                cerr << "[Error] Unknown option: '" << option_name << "'" << endl;
                is_print_options = true;
                break;
            }
        }

        // Check if all required options are set (the ones without default values)
        vector<String>::const_iterator it;
        for(it = m_option_names_.begin(); it != m_option_names_.end(); ++it)
        {
            const String& option_name = *it;
            const OptionInfo* option_info = m_option_infos_.get(option_name);

            if(!option_info->hasDefaultArgValue())
            {
                const String& var_name = option_info->getVarName();
                if(!keyExist(var_name))
                {
                    cerr << "[Error] Missing required option: '" << option_name << "'" << endl;
                    is_print_options = true;
                }
            }
        }

        if(is_print_options)
        {
            printOptions();
            exit(0);
        }
        return;
    }

    void OptionParser::printOptions() const
    {
        cout << endl;
        cout << "Available options:" << endl;
        cout << "==================" << endl << endl;

        vector<String>::const_iterator it;
        for(it = m_option_names_.begin(); it != m_option_names_.end(); ++it)
        {
            const String& option_name = *it;
            const OptionInfo* option_info = m_option_infos_.get(option_name);

            cout << option_name;
            if(option_info->hasArg())
            {
                cout << " <" << option_info->getArgName() << ">";
            }
            cout << endl;

            cout << "    " << option_info->getDescription() << endl;
            if(option_info->hasArg() && option_info->hasDefaultArgValue())
            {
                cout << "    " << "Default: " << option_info->getDefaultArgValue() << endl;
            }
            cout << endl;
        }
        cout << "-help" << endl;
        cout << "    " << "Print this page" << endl;
        cout << endl;
        return;
    }

} // namespace LibUtil
