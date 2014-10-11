#ifndef __DSENT_UTIL_COMMON_TYPE_H__
#define __DSENT_UTIL_COMMON_TYPE_H__

#include <iostream>

#include "libutil/LibUtil.h"

#include "util/Result.h"

#include "tech/TechModel.h"

namespace DSENT
{
    using std::cout;
    using std::endl;

    // Enable functions
    using LibUtil::deletePtrMap;
    using LibUtil::clearPtrMap;
    using LibUtil::deletePtrVector;
    using LibUtil::clearPtrVector;

    // Enable classes
    using LibUtil::Exception;
    using LibUtil::Log;
    using LibUtil::String;
    using LibUtil::Map;
    using LibUtil::StringMap;

    typedef StringMap   ParameterMap;
    typedef StringMap   PropertyMap;

} // namespace DSENT

#endif // __DSENT_UTIL_COMMON_TYPE_H__

