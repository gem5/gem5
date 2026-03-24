# src/base/loader/CMakeLists.txt
# Converted from src/base/loader/SConscript

gem5_add_source(dtb_file.cc)
gem5_add_source(elf_object.cc)
gem5_add_source(image_file_data.cc)
gem5_add_source(memory_image.cc)
gem5_add_source(object_file.cc)
gem5_add_source(symtab.cc)

# --- Unit tests ---
gem5_add_test_source(image_file_data.test.cc)
gem5_add_test_source(symtab.test.cc)
