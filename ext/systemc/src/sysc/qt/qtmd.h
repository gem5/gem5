#if defined( __sparc ) || defined( __sparc__ )
#include "sysc/qt/md/sparc.h"
#elif defined( __hppa )
#include "sysc/qt/md/hppa.h"
#elif defined( __x86_64__ )
#include "sysc/qt/md/iX86_64.h"
#elif defined( __i386 )
#include "sysc/qt/md/i386.h"
#elif defined( __ppc__ )
#include "sysc/qt/md/powerpc_mach.h"
#elif defined( __powerpc )
#include "sysc/qt/md/powerpc_sys5.h"
#endif
