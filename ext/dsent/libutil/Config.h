#ifndef __LIBUTIL_CONFIG_H__
#define __LIBUTIL_CONFIG_H__

#include <iostream>

#include "Map.h"

namespace LibUtil
{
    class Config : public StringMap
    {
        public:
            Config(const String& delimiter_ = "=", const String& comment_ = "#", const String& sentry_ = "End");
            Config(const Config& config_);
            virtual ~Config();

        public:
            // Make a copy of this instance
            virtual Config* clone() const;
            // Load the config from file
            virtual void readFile(const String& filename_);
            // Parse string and overwrite the Config instance if keys exist
            virtual void readString(const String& str_);

            // Write or read map using standard IO
            friend std::ostream& operator<<(std::ostream& ost_, const Config& config_);
            friend std::istream& operator>>(std::istream& ist_, Config& config_);

        protected:
            String mDelimiter;
            String mComment;
            String mSentry;
    };
}

#endif // __LIBUTIL_CONFIG_H__

