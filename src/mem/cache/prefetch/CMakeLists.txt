# src/mem/cache/prefetch/CMakeLists.txt
# Converted from src/mem/cache/prefetch/SConscript

gem5_add_source(access_map_pattern_matching.cc)
gem5_add_source(base.cc)
gem5_add_source(bop.cc)
gem5_add_source(delta_correlating_prediction_tables.cc)
gem5_add_source(fdp.cc)
gem5_add_source(indirect_memory.cc)
gem5_add_source(irregular_stream_buffer.cc)
gem5_add_source(multi.cc)
gem5_add_source(pif.cc)
gem5_add_source(queued.cc)
gem5_add_source(sbooe.cc)
gem5_add_source(signature_path.cc)
gem5_add_source(signature_path_v2.cc)
gem5_add_source(slim_ampm.cc)
gem5_add_source(sms.cc)
gem5_add_source(spatio_temporal_memory_streaming.cc)
gem5_add_source(stride.cc)
gem5_add_source(tagged.cc)

# SimObjects
gem5_add_simobject(Prefetcher.py
    SIM_OBJECTS
        BasePrefetcher MultiPrefetcher QueuedPrefetcher
        StridePrefetcherHashedSetAssociative StridePrefetcher
        SmsPrefetcher TaggedPrefetcher IndirectMemoryPrefetcher
        SignaturePathPrefetcher SignaturePathPrefetcherV2
        AccessMapPatternMatching AMPMPrefetcher
        DeltaCorrelatingPredictionTables DCPTPrefetcher
        IrregularStreamBufferPrefetcher SlimAMPMPrefetcher
        BOPPrefetcher SBOOEPrefetcher STeMSPrefetcher PIFPrefetcher
        FetchDirectedPrefetcher)
