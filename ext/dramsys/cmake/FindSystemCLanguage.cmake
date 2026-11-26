if (NOT SystemCLanguage_FOUND)
    add_library(systemc INTERFACE)

    target_include_directories(systemc INTERFACE
        "$<BUILD_INTERFACE:${SCONS_SOURCE_DIR}/src/systemc/ext/systemc_home/include>"
        "$<INSTALL_INTERFACE:include>"
    )

    add_library(SystemC::systemc ALIAS systemc)

    set(SystemCLanguage_FOUND TRUE)

    install(
        TARGETS systemc
        EXPORT DRAMSysTargets
    )
endif()
