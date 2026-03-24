# src/dev/qemu/CMakeLists.txt
# Converted from src/dev/qemu/SConscript

gem5_add_source(fw_cfg.cc)

gem5_add_debug_flag(QemuFwCfg "QemuFwCfg debugging")
gem5_add_debug_flag(QemuFwCfgVerbose "QemuFwCfgVerbose debugging")

gem5_add_simobject(QemuFwCfg.py SIM_OBJECTS
    QemuFwCfgItem QemuFwCfgItemBytes QemuFwCfgItemFile QemuFwCfgItemString
    QemuFwCfg QemuFwCfgIo QemuFwCfgMmio)
