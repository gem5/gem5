#include "Log.h"

#include "Assert.h"

namespace LibUtil
{
    using std::ostream;
    using std::endl;

    Log* Log::msSingleton = NULL;
    const bool Log::msIsLog = LIBUTIL_IS_LOG;

    void Log::allocate(const String& log_file_name_)
    {
        if(msIsLog)
        {
            // Allocate static Config instance
            ASSERT(!msSingleton, "Log singleton is allocated");
            msSingleton = new Log(log_file_name_);
        }
    }

    void Log::release()
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            delete msSingleton;
            msSingleton = NULL;
        }
        return;
    }

    void Log::print(const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_;
        }
        return;
    }

    void Log::print(ostream& stream_, const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_;
        }
        stream_ << str_;
        return;
    }

    void Log::printLine(const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_ << endl;
        }
        return;
    }

    void Log::printLine(ostream& stream_, const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_ << endl;
        }
        stream_ << str_ << endl;
        return;
    }

    Log::Log(const String& log_file_name_)
    {
        ofs.open(log_file_name_.c_str());
    }

    Log::~Log()
    {
        ofs.close();
    }
}

