#ifndef __ASSERT_H__
#define __ASSERT_H__

#include "String.h"
#include "Exception.h"

#ifdef NDEBUG
#define ASSERT(test_value_,exception_msg_)
#else
#define ASSERT(test_value_,msg_) \
    do \
    { \
        if(!(test_value_)) \
        { \
            const LibUtil::String& exception_msg = LibUtil::String::format("\nAt %s:%d\n", __FILE__, __LINE__) + (String)(msg_); \
            throw LibUtil::Exception(exception_msg); \
        } \
    } while(0);
#endif

#endif // __ASSERT_H__

