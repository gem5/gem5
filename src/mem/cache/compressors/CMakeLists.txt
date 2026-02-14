# src/mem/cache/compressors/CMakeLists.txt
# Converted from src/mem/cache/compressors/SConscript

gem5_add_source(base.cc)
gem5_add_source(base_delta.cc)
gem5_add_source(base_dictionary_compressor.cc)
gem5_add_source(cpack.cc)
gem5_add_source(fpc.cc)
gem5_add_source(fpcd.cc)
gem5_add_source(frequent_values.cc)
gem5_add_source(multi.cc)
gem5_add_source(perfect.cc)
gem5_add_source(repeated_qwords.cc)
gem5_add_source(zero.cc)

add_subdirectory(encoders)

# SimObjects
gem5_add_simobject(Compressors.py
    SIM_OBJECTS
        BaseCacheCompressor BaseDictionaryCompressor
        Base64Delta8 Base64Delta16 Base64Delta32
        Base32Delta8 Base32Delta16 Base16Delta8
        CPack FPC FPCD FrequentValuesCompressor MultiCompressor
        PerfectCompressor RepeatedQwordsCompressor ZeroCompressor)
