#include "mem/cache/CacheSelector.hh"
// cache_selector.cc

#include "sim/sim_object.hh"
#include <iostream>
// #include "base/inc/RegisterSimObject.h"

namespace gem5
{
    CacheSelector::CacheSelector(const CacheSelectorParams &p) : SimObject(p) {
        // 构造函数实现
        std::cout << "hello, my cart!!!\n";
    }
}
