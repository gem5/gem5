#ifndef __ALIGNED_HH__
#define __ALIGNED_HH__

#include <stdint.h>
#include "targetarch/isa_traits.hh"

/* GCC 3.3.X has a bug in which attributes+typedefs don't work. 3.2.X is fine
 * as in 3.4.X, but the bug is marked will not fix in 3.3.X so here is
 * the work around.
 */
#if __GNUC__ == 3 && __GNUC_MINOR__  != 3
typedef uint64_t uint64_ta __attribute__ ((aligned (8))) ;
typedef int64_t int64_ta __attribute__ ((aligned (8))) ;
typedef Addr Addr_a __attribute__ ((aligned (8))) ;
#else
#define uint64_ta uint64_t __attribute__ ((aligned (8)))
#define int64_ta int64_t __attribute__ ((aligned (8)))
#define Addr_a Addr __attribute__ ((aligned (8)))
#endif /* __GNUC__ __GNUC_MINOR__ */

#endif /* __ALIGNED_H__ */
