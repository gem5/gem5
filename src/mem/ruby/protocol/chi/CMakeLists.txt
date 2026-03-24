# src/mem/ruby/protocol/chi/CMakeLists.txt
# Conditional on CONF_RUBY_PROTOCOL_CHI (guarded by parent)

add_subdirectory(generic)

if(CONF_BUILD_TLM)
    add_subdirectory(tlm)
endif()
