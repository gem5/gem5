add_library(gem5_ext_drampower STATIC
    src/CommandAnalysis.cc
    src/MemArchitectureSpec.cc
    src/MemCommand.cc
    src/MemPowerSpec.cc
    src/MemTimingSpec.cc
    src/MemoryPowerModel.cc
    src/MemorySpecification.cc
    src/Parameter.cc
    src/Parametrisable.cc
    src/libdrampower/LibDRAMPower.cc
    src/CAHelpers.cc
    src/CmdHandlers.cc
    src/MemBankWiseParams.cc
)

target_include_directories(gem5_ext_drampower SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set_target_properties(gem5_ext_drampower PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
