# src/dev/virtio/CMakeLists.txt
# Converted from src/dev/virtio/SConscript

gem5_add_source(base.cc)
gem5_add_source(pci.cc)
gem5_add_source(console.cc)
gem5_add_source(block.cc)
gem5_add_source(fs9p.cc)
gem5_add_source(rng.cc)

gem5_add_debug_flag(VIO "VirtIO base functionality")
gem5_add_debug_flag(VIORng "VirtIO entropy source device")
gem5_add_debug_flag(VIOIface "VirtIO transport")
gem5_add_debug_flag(VIOConsole "VirtIO console device")
gem5_add_debug_flag(VIOBlock "VirtIO block device")
gem5_add_debug_flag(VIO9P "General 9p over VirtIO debugging")
gem5_add_debug_flag(VIO9PData "Dump data in VirtIO 9p connections")

gem5_add_simobject(VirtIO.py SIM_OBJECTS
    VirtIODeviceBase VirtIODummyDevice PciVirtIO)
gem5_add_simobject(VirtIOConsole.py SIM_OBJECTS VirtIOConsole)
gem5_add_simobject(VirtIOBlock.py SIM_OBJECTS VirtIOBlock)
gem5_add_simobject(VirtIORng.py SIM_OBJECTS VirtIORng)
gem5_add_simobject(VirtIO9P.py SIM_OBJECTS
    VirtIO9PBase VirtIO9PProxy VirtIO9PDiod VirtIO9PSocket)
