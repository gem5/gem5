/* $Id$ */

#ifndef __EV5_H__
#define __EV5_H__

#ifndef SYSTEM_EV5
#error This code is only valid for EV5 systems
#endif

#include "targetarch/isa_traits.hh"

////////////////////////////////////////////////////////////////////////
//
//
//

////////////////////////////////////////////////////////////////////////
//
//
//

#define MODE2MASK(X)			(1 << (X))

// Alpha IPR register accessors
#define PC_PAL(X)			((X) & 0x1)
#define MCSR_SP(X)			(((X) >> 1) & 0x3)

#define ICSR_SDE(X)			(((X) >> 30) & 0x1)
#define ICSR_SPE(X)			(((X) >> 28) & 0x3)
#define ICSR_FPE(X)			(((X) >> 26) & 0x1)

#define ALT_MODE_AM(X)			(((X) >> 3) & 0x3)

#define DTB_CM_CM(X)			(((X) >> 3) & 0x3)

#ifdef ALPHA_TLASER
#define DTB_ASN_ASN(X)          (((X) >> 57) & 0x7f)
#define DTB_PTE_PPN(X)          (((X) >> 32) & 0x07ffffff)
#else
#define DTB_ASN_ASN(X)			(((X) >> 57) & 0xff)
#define DTB_PTE_PPN(X)			(((X) >> 32) & 0x07fffffff)
#endif

#define DTB_PTE_XRE(X)			(((X) >> 8) & 0xf)
#define DTB_PTE_XWE(X)			(((X) >> 12) & 0xf)
#define DTB_PTE_FONR(X)			(((X) >> 1) & 0x1)
#define DTB_PTE_FONW(X)			(((X) >> 2) & 0x1)
#define DTB_PTE_GH(X)			(((X) >> 5) & 0x3)
#define DTB_PTE_ASMA(X)			(((X) >> 4) & 0x1)

#define ICM_CM(X)				(((X) >> 3) & 0x3)

#ifdef ALPHA_TLASER
#define ITB_ASN_ASN(X)          (((X) >> 4) & 0x7f)
#define ITB_PTE_PPN(X)          (((X) >> 32) & 0x07ffffff)
#else
#define ITB_ASN_ASN(X)			(((X) >> 4) & 0xff)
#define ITB_PTE_PPN(X)			(((X) >> 32) & 0x07fffffff)
#endif

#define ITB_PTE_XRE(X)			(((X) >> 8) & 0xf)
#define ITB_PTE_FONR(X)			(((X) >> 1) & 0x1)
#define ITB_PTE_FONW(X)			(((X) >> 2) & 0x1)
#define ITB_PTE_GH(X)			(((X) >> 5) & 0x3)
#define ITB_PTE_ASMA(X)			(((X) >> 4) & 0x1)

#define VA_UNIMPL_MASK			ULL(0xfffff80000000000)
#define VA_IMPL_MASK			ULL(0x000007ffffffffff)
#define VA_IMPL(X)				((X) & VA_IMPL_MASK)
#define VA_VPN(X)				(VA_IMPL(X) >> 13)
#define VA_SPACE_EV5(X)			(((X) >> 41) & 0x3)
#define VA_SPACE_EV6(X) 	    (((X) >> 41) & 0x7f)
#define VA_POFS(X)				((X) & 0x1fff)

#define PA_UNCACHED_BIT_39		ULL(0x8000000000)
#define PA_UNCACHED_BIT_40		ULL(0x10000000000)
#define PA_UNCACHED_BIT_43		ULL(0x80000000000)
#define PA_UNCACHED_MASK                ULL(0x807ffffffff) // Clear PA<42:35>
#ifdef ALPHA_TLASER
#define PA_IPR_SPACE(X)         ((X) >= ULL(0xFFFFF00000))
#define PA_IMPL_MASK			ULL(0xffffffffff)
#else
#define PA_IPR_SPACE(X)			((X) >= ULL(0xFFFFFF00000))
#define PA_IMPL_MASK			ULL(0xfffffffffff) // for Tsunami
#endif

#define PA_PFN2PA(X)			((X) << 13)


#define MM_STAT_BAD_VA_MASK		0x0020
#define MM_STAT_DTB_MISS_MASK		0x0010
#define MM_STAT_FONW_MASK		0x0008
#define MM_STAT_FONR_MASK		0x0004
#define MM_STAT_ACV_MASK		0x0002
#define MM_STAT_WR_MASK			0x0001

#define OPCODE(X)                       (X >> 26) & 0x3f
#define RA(X)                           (X >> 21) & 0x1f

////////////////////////////////////////////////////////////////////////
//
//
//

// VPTE size for HW_LD/HW_ST
#define HW_VPTE		((inst >> 11) & 0x1)

// QWORD size for HW_LD/HW_ST
#define HW_QWORD	((inst >> 12) & 0x1)

// ALT mode for HW_LD/HW_ST
#define HW_ALT		(((inst >> 14) & 0x1) ? ALTMODE : 0)

// LOCK/COND mode for HW_LD/HW_ST
#define HW_LOCK		(((inst >> 10) & 0x1) ? LOCKED : 0)
#define HW_COND		(((inst >> 10) & 0x1) ? LOCKED : 0)

// PHY size for HW_LD/HW_ST
#define HW_PHY		(((inst >> 15) & 0x1) ? PHYSICAL : 0)

// OFFSET for HW_LD/HW_ST
#define HW_OFS		(inst & 0x3ff)


#define PAL_BASE 0x4000

#endif //__EV5_H__
