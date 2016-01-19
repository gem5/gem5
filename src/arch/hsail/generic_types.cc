#include "arch/hsail/generic_types.hh"
#include "base/misc.hh"

using namespace Brig;

namespace HsailISA
{
    Enums::GenericMemoryOrder
    getGenericMemoryOrder(BrigMemoryOrder brig_memory_order)
    {
        switch(brig_memory_order) {
          case BRIG_MEMORY_ORDER_NONE:
            return Enums::MEMORY_ORDER_NONE;
          case BRIG_MEMORY_ORDER_RELAXED:
            return Enums::MEMORY_ORDER_RELAXED;
          case BRIG_MEMORY_ORDER_SC_ACQUIRE:
            return Enums::MEMORY_ORDER_SC_ACQUIRE;
          case BRIG_MEMORY_ORDER_SC_RELEASE:
            return Enums::MEMORY_ORDER_SC_RELEASE;
          case BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE:
            return Enums::MEMORY_ORDER_SC_ACQUIRE_RELEASE;
          default:
            fatal("HsailISA::MemInst::getGenericMemoryOrder -> ",
                  "bad BrigMemoryOrder\n");
        }
    }

    Enums::GenericMemoryScope
    getGenericMemoryScope(BrigMemoryScope brig_memory_scope)
    {
        switch(brig_memory_scope) {
          case BRIG_MEMORY_SCOPE_NONE:
            return Enums::MEMORY_SCOPE_NONE;
          case BRIG_MEMORY_SCOPE_WORKITEM:
            return Enums::MEMORY_SCOPE_WORKITEM;
          case BRIG_MEMORY_SCOPE_WORKGROUP:
            return Enums::MEMORY_SCOPE_WORKGROUP;
          case BRIG_MEMORY_SCOPE_AGENT:
            return Enums::MEMORY_SCOPE_DEVICE;
          case BRIG_MEMORY_SCOPE_SYSTEM:
            return Enums::MEMORY_SCOPE_SYSTEM;
          default:
            fatal("HsailISA::MemInst::getGenericMemoryScope -> ",
                  "bad BrigMemoryScope\n");
        }
    }
} // namespace HsailISA
