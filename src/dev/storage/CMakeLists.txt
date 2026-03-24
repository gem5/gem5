# src/dev/storage/CMakeLists.txt
# Converted from src/dev/storage/SConscript

gem5_add_source(ide_ctrl.cc)
gem5_add_source(ide_disk.cc)
gem5_add_source(disk_image.cc)
gem5_add_source(simple_disk.cc)

gem5_add_debug_flag(IdeCtrl "IdeCtrl debugging")
gem5_add_debug_flag(IdeDisk "IdeDisk debugging")
gem5_add_debug_flag(DiskImageRead "DiskImageRead debugging")
gem5_add_debug_flag(DiskImageWrite "DiskImageWrite debugging")
gem5_add_debug_flag(SimpleDisk "SimpleDisk debugging")
gem5_add_debug_flag(SimpleDiskData "SimpleDiskData debugging")

# Compound flags
gem5_add_debug_flag(DiskImageAll "All DiskImage debugging"
    COMPONENTS DiskImageRead DiskImageWrite)
gem5_add_debug_flag(IdeAll "All IDE debugging"
    COMPONENTS IdeCtrl IdeDisk)

gem5_add_simobject(Ide.py SIM_OBJECTS IdeDisk IdeController ENUMS IdeID)
gem5_add_simobject(DiskImage.py SIM_OBJECTS DiskImage RawDiskImage CowDiskImage)
gem5_add_simobject(SimpleDisk.py SIM_OBJECTS SimpleDisk)
