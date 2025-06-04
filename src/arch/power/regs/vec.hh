#ifndef __ARCH_POWER_REGS_VEC_HH__
#define __ARCH_POWER_REGS_VEC_HH__

#include "cpu/reg_class.hh"

//#include "debug/VecRegs.hh"
#include "debug/IntRegs.hh"

namespace gem5
{
namespace PowerISA
{

namespace vsr_reg
{
    // Primary definition as constexpr
    constexpr unsigned NumRegs = 64;
    constexpr unsigned FPR_OFFSET = 32; // VSR32-63 overlap with FPR0-31

    enum : RegIndex
    {
    _V0Idx, _V1Idx, _V2Idx, _V3Idx, _V4Idx, _V5Idx, _V6Idx, _V7Idx,
    _V8Idx, _V9Idx, _V10Idx, _V11Idx, _V12Idx, _V13Idx, _V14Idx, _V15Idx,
    _V16Idx, _V17Idx, _V18Idx, _V19Idx, _V20Idx, _V21Idx, _V22Idx, _V23Idx,
    _V24Idx, _V25Idx, _V26Idx, _V27Idx, _V28Idx, _V29Idx, _V30Idx, _V31Idx,
    _V32Idx, _V33Idx, _V34Idx, _V35Idx, _V36Idx, _V37Idx, _V38Idx, _V39Idx,
    _V40Idx, _V41Idx, _V42Idx, _V43Idx, _V44Idx, _V45Idx, _V46Idx, _V47Idx,
    _V48Idx, _V49Idx, _V50Idx, _V51Idx, _V52Idx, _V53Idx, _V54Idx, _V55Idx,
    _V56Idx, _V57Idx, _V58Idx, _V59Idx, _V60Idx, _V61Idx, _V62Idx, _V63Idx,

    _NumRegsEnum    //Renamed to avoid conflict
};

} // namespace vsr_reg

inline constexpr RegClass vecRegClass(VecRegClass, "vsr",
                                    vsr_reg::_NumRegsEnum,
                                    debug::IntRegs);    //Placeholder

namespace vsr_reg
{
   inline constexpr RegId
    V0 = vecRegClass[_V0Idx],
    V1 = vecRegClass[_V1Idx],
    V2 = vecRegClass[_V2Idx],
    V3 = vecRegClass[_V3Idx],
    V4 = vecRegClass[_V4Idx],
    V5 = vecRegClass[_V5Idx],
    V6 = vecRegClass[_V6Idx],
    V7 = vecRegClass[_V7Idx],
    V8 = vecRegClass[_V8Idx],
    V9 = vecRegClass[_V9Idx],
    V10 = vecRegClass[_V10Idx],
    V11 = vecRegClass[_V11Idx],
    V12 = vecRegClass[_V12Idx],
    V13 = vecRegClass[_V13Idx],
    V14 = vecRegClass[_V14Idx],
    V15 = vecRegClass[_V15Idx],
    V16 = vecRegClass[_V16Idx],
    V17 = vecRegClass[_V17Idx],
    V18 = vecRegClass[_V18Idx],
    V19 = vecRegClass[_V19Idx],
    V20 = vecRegClass[_V20Idx],
    V21 = vecRegClass[_V21Idx],
    V22 = vecRegClass[_V22Idx],
    V23 = vecRegClass[_V23Idx],
    V24 = vecRegClass[_V24Idx],
    V25 = vecRegClass[_V25Idx],
    V26 = vecRegClass[_V26Idx],
    V27 = vecRegClass[_V27Idx],
    V28 = vecRegClass[_V28Idx],
    V29 = vecRegClass[_V29Idx],
    V30 = vecRegClass[_V30Idx],
    V31 = vecRegClass[_V31Idx],
    V32 = vecRegClass[_V32Idx],
    V33 = vecRegClass[_V33Idx],
    V34 = vecRegClass[_V34Idx],
    V35 = vecRegClass[_V35Idx],
    V36 = vecRegClass[_V36Idx],
    V37 = vecRegClass[_V37Idx],
    V38 = vecRegClass[_V38Idx],
    V39 = vecRegClass[_V39Idx],
    V40 = vecRegClass[_V40Idx],
    V41 = vecRegClass[_V41Idx],
    V42 = vecRegClass[_V42Idx],
    V43 = vecRegClass[_V43Idx],
    V44 = vecRegClass[_V44Idx],
    V45 = vecRegClass[_V45Idx],
    V46 = vecRegClass[_V46Idx],
    V47 = vecRegClass[_V47Idx],
    V48 = vecRegClass[_V48Idx],
    V49 = vecRegClass[_V49Idx],
    V50 = vecRegClass[_V50Idx],
    V51 = vecRegClass[_V51Idx],
    V52 = vecRegClass[_V52Idx],
    V53 = vecRegClass[_V53Idx],
    V54 = vecRegClass[_V54Idx],
    V55 = vecRegClass[_V55Idx],
    V56 = vecRegClass[_V56Idx],
    V57 = vecRegClass[_V57Idx],
    V58 = vecRegClass[_V58Idx],
    V59 = vecRegClass[_V59Idx],
    V60 = vecRegClass[_V60Idx],
    V61 = vecRegClass[_V61Idx],
    V62 = vecRegClass[_V62Idx],
    V63 = vecRegClass[_V63Idx];

} // namespace vsr_reg

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_REGS_VEC_HH__
