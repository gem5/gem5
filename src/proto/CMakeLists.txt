# src/proto/CMakeLists.txt
#
# Protobuf compilation for gem5 trace files.
# Generates .pb.cc and .pb.h from .proto definitions.
#
# The generated headers are included as "proto/<name>.pb.h" from gem5 source
# code, so we generate them into ${GEM5_GEN_DIR}/proto/ which is under the
# generated include root (${GEM5_GEN_DIR}).
#
# Custom commands are created centrally at the top level (not here) to avoid
# CMake/Ninja cross-directory custom command stub issues. We store metadata
# in global properties; gem5_create_proto_commands() creates the commands.

if(HAVE_PROTOBUF)
    set(_proto_dir "${CMAKE_CURRENT_SOURCE_DIR}")
    set(_proto_out "${GEM5_GEN_DIR}/proto")
    file(MAKE_DIRECTORY "${_proto_out}")

    set(_proto_files
        inst.proto
        inst_dep_record.proto
        packet.proto
    )

    # Suppress warnings in protoc-generated code
    if(GEM5_GCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "10.0")
        set(_proto_suppress_flags "-Wno-array-bounds;-Wno-stringop-overflow")
    else()
        set(_proto_suppress_flags "-Wno-array-bounds")
    endif()

    # Store suppress flags for use by gem5_create_proto_commands()
    set_property(GLOBAL PROPERTY GEM5_PROTO_SUPPRESS_FLAGS "${_proto_suppress_flags}")

    foreach(_proto ${_proto_files})
        # Store .proto absolute path for deferred command creation
        set_property(GLOBAL APPEND PROPERTY GEM5_PROTO_FILES
            "${_proto_dir}/${_proto}")

        # Register generated .cc as source (command created later at top level)
        get_filename_component(_name "${_proto}" NAME_WE)
        gem5_add_generated_source("${_proto_out}/${_name}.pb.cc")
    endforeach()

    # Static protobuf support sources
    gem5_add_sources(protobuf.cc protoio.cc)

    # Link against protobuf library (consumed when building the gem5 target)
    set_property(GLOBAL APPEND PROPERTY GEM5_LINK_LIBRARIES
        protobuf::libprotobuf)
endif()
