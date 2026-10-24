#include <stdint.h>

enum TaskEvent : uint64_t
{
    TASK_BEGIN = 0,
    TASK_END   = 1,
    STEAL_TRY  = 2,
    STEAL_OK   = 3
};
