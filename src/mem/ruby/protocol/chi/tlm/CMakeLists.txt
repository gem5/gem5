# src/mem/ruby/protocol/chi/tlm/CMakeLists.txt
# Converted from src/mem/ruby/protocol/chi/tlm/SConscript
# Conditional on CONF_BUILD_TLM (guarded by parent)

# TODO: TLM integration requires external libarmtlmchi.so
# The SCons build adds CPPPATH, LIBPATH, LIBS for the TLM library.
# This will need proper find_library() / target_link_libraries() setup.

gem5_add_source(controller.cc)
gem5_add_source(generator.cc)
gem5_add_source(utils.cc)
gem5_add_source(tlm_chi.cc)
gem5_add_source(tlm_chi_gen.cc)

gem5_add_debug_flag(TLM "TLM debugging")
gem5_add_debug_flag(TLMPort "TLMPort debugging")

# SimObject (requires gem5py, deferred)
# SimObject('TlmController.py', ...) SimObject('TlmGenerator.py', ...)
