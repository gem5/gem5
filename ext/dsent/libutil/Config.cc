#include "Config.h"

#include <fstream>

#include "Assert.h"

namespace LibUtil
{
    Config::Config(const String& delimiter_, const String& comment_, const String& sentry_)
        : mDelimiter(delimiter_), mComment(comment_), mSentry(sentry_)
    {}

    Config::Config(const Config& config_)
        : StringMap(config_)
    {
        mDelimiter = config_.mDelimiter;
        mComment = config_.mComment;
        mSentry = config_.mSentry;
    }

    Config::~Config()
    {}

    Config* Config::clone() const
    {
        return new Config(*this);
    }

    void Config::readFile(const String& filename_)
    {
        std::ifstream fin(filename_.c_str());

        ASSERT(fin, "File not found: " + filename_);
        fin >> (*this);
        return;
    }

    void Config::readString(const String& str_)
    {
        String newString = str_;
        newString.substitute(";", "\n");
        std::istringstream iss(newString, std::istringstream::in);

        iss >> (*this);
    }

    std::ostream& operator<<(std::ostream& ost_, const Config& config_)
    {
        Config::ConstIterator it;
        for(it = config_.begin(); it != config_.end(); it++)
        {
            ost_ << it->first << " " << config_.mDelimiter << " ";
            ost_ << it->second << std::endl;
        }
        return ost_;
    }

    std::istream& operator>>(std::istream& ist_, Config& config_)
    {
        // Set a Config from ist_
        // Read in keys and values, keeping internal whitespace
        typedef String::size_type pos;
        const String& delim  = config_.mDelimiter;  // separator
        const String& comm   = config_.mComment;    // comment
        const String& sentry = config_.mSentry;     // end of file sentry
        const pos skip = delim.length();        // length of separator

        String nextline = "";  // might need to read ahead to see where value ends

        while(ist_ || nextline.length() > 0)
        {
            // Read an entire line at a time
            String line;
            if(nextline.length() > 0)
            {
                line = nextline;  // we read ahead; use it now
                nextline = "";
            }
            else
            {
                //std::getline(ist_, line);
                safeGetline(ist_, line);
            }

            // Ignore comments and the spaces on both ends
            line = line.substr(0, line.find(comm));
            line.trim();

            // Check for end of file sentry
            if((sentry != "") && (line.find(sentry) != String::npos)) return ist_;

            if(line.length() == 0)
                continue;

            // Parse the line if it contains a delimiter
            pos delimPos = line.find(delim);
            ASSERT((delimPos < String::npos), "Invalid config line: '" + line + "'");

            // Extract the key
            String key = line.substr(0, delimPos);
            line.replace(0, delimPos+skip, "");

            // See if value continues on the next line
            // Stop at blank line, next line with a key, end of stream,
            // or end of file sentry
            bool terminate = false;
            while(!terminate && ist_)
            {
                if(line.at(line.size() - 1) == '\\')
                    line.erase(line.size() - 1);
                else
                    break;

                //std::getline(ist_, nextline);
                safeGetline(ist_, nextline);
                terminate = true;

                String nlcopy = nextline;
                nlcopy.trim();
                if(nlcopy == "") continue;

                nextline = nextline.substr(0, nextline.find(comm));
                //if(nextline.find(delim) != String::npos)
                //    continue;
                if((sentry != "") && (nextline.find(sentry) != String::npos))
                    continue;

                //nlcopy = nextline;
                //nlcopy.trim();
                //if(nlcopy != "") line += "\n";
                line += nextline;
                nextline = "";
                terminate = false;
            }

            // Store key and value
            key.trim();
            line.trim();
            config_.set(key, line);  // overwrites if key is repeated
        }
        return ist_;
    }
}

