#ifndef __LIBUTIL_OPTION_PARSER_H__
#define __LIBUTIL_OPTION_PARSER_H__

#include <vector>

#include "Map.h"

namespace LibUtil
{
    using std::vector;

    // Simple option parser
    class OptionParser : public StringMap
    {
        private:
            class OptionInfo
            {
                public:
                    OptionInfo(const String& var_name_, bool has_arg_, const String& arg_name_, bool has_default_arg_value_, const String& default_arg_value_, const String& description_);
                    ~OptionInfo();

                public:
                    inline const String& getVarName() const { return m_var_name_; }
                    inline bool hasArg() const { return m_has_arg_; }
                    inline const String& getArgName() const { return m_arg_name_; }
                    inline bool hasDefaultArgValue() const { return m_has_default_arg_value_; }
                    inline const String& getDefaultArgValue() const { return m_default_arg_value_; }
                    inline const String& getDescription() const { return m_description_; }

                private:
                    String m_var_name_;
                    bool m_has_arg_;
                    String m_arg_name_;
                    bool m_has_default_arg_value_;
                    String m_default_arg_value_;
                    String m_description_;
            }; // class Option

        public:
            OptionParser();
            virtual ~OptionParser();

        public:
            void addOption(const String& option_name_, const String& var_name_, bool has_arg_, const String& arg_name_, bool has_default_arg_value_, const String& default_arg_value_, const String& description_);

            void parseArguments(int argc_, char** argv_);

            void printOptions() const;

        protected:
            vector<String> m_option_names_;
            Map<OptionInfo*> m_option_infos_;
    }; // class OptionParser
} // LibUtil

#endif // __LIBUTIL_OPTION_PARSER_H__

