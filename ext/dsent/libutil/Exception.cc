#include "Exception.h"

namespace LibUtil
{
    Exception::Exception(const String& exception_msg_) throw()
        : exception(), mExceptionMsg(exception_msg_)
    {}

    Exception::~Exception() throw()
    {}

    const char* Exception::what() const throw()
    {
        return mExceptionMsg.c_str();
    }
}

