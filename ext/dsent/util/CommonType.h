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

