#ifndef __ARCH_HSAIL_GENERIC_TYPES_HH__
#define __ARCH_HSAIL_GENERIC_TYPES_HH__

#include "arch/hsail/Brig.h"
#include "enums/GenericMemoryOrder.hh"
#include "enums/GenericMemoryScope.hh"

namespace HsailISA
{
    Enums::GenericMemoryOrder
    getGenericMemoryOrder(Brig::BrigMemoryOrder brig_memory_order);
    Enums::GenericMemoryScope
    getGenericMemoryScope(Brig::BrigMemoryScope brig_memory_scope);
} // namespace HsailISA

#endif // __ARCH_HSAIL_GENERIC_TYPES_HH__
