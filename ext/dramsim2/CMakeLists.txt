# DRAMSim2 is optional: only built if the DRAMSim2/ subdirectory has been
# cloned into ext/dramsim2/.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/DRAMSim2")
    set(HAVE_DRAMSIM OFF PARENT_SCOPE)
    message(STATUS "ext/dramsim2: DRAMSim2/ not found, skipping")
    return()
endif()

set(HAVE_DRAMSIM ON PARENT_SCOPE)
message(STATUS "ext/dramsim2: DRAMSim2/ found, building")

add_library(gem5_ext_dramsim2 STATIC
    DRAMSim2/AddressMapping.cpp
    DRAMSim2/Bank.cpp
    DRAMSim2/BankState.cpp
    DRAMSim2/BusPacket.cpp
    DRAMSim2/ClockDomain.cpp
    DRAMSim2/CommandQueue.cpp
    DRAMSim2/IniReader.cpp
    DRAMSim2/MemoryController.cpp
    DRAMSim2/MemorySystem.cpp
    DRAMSim2/MultiChannelMemorySystem.cpp
    DRAMSim2/Rank.cpp
    DRAMSim2/SimulatorObject.cpp
    DRAMSim2/Transaction.cpp
)

target_include_directories(gem5_ext_dramsim2 SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(gem5_ext_dramsim2 PRIVATE
    NO_STORAGE
)

target_compile_options(gem5_ext_dramsim2 PRIVATE
    -Wno-unused-value
    $<$<CXX_COMPILER_ID:GNU>:
        -Wno-nonnull-compare
    >
    $<$<CXX_COMPILER_ID:Clang>:
        -Wno-unused-private-field
        -Wno-tautological-undefined-compare
    >
    $<$<CXX_COMPILER_ID:AppleClang>:
        -Wno-unused-private-field
        -Wno-tautological-undefined-compare
    >
)

set_target_properties(gem5_ext_dramsim2 PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
