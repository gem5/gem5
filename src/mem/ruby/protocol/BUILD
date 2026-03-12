load("@rules_python//python:defs.bzl", "py_library")
load("//src/mem/ruby/protocol:slicc.bzl", "default_protocol_cc_deps", "default_sim_object_param_deps", "default_sim_object_py_deps", "gem5_slicc_cc_library", "generate_imported_headers", "generate_shared_slicc")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "protocol_init",
    srcs = ["__init__.py"],
    deps = ["//src/mem/ruby:ruby_init"],
)

# --------------------------------------------
# Shared protocol files
# --------------------------------------------
# These files are shared by all protocols

# These are files that are created by slicc when processing the shared RubySlicc_interfaces.slicc
shared_outs = [
    "AccessPermission.cc",
    "AccessPermission.hh",
    "AccessType.cc",
    "AccessType.hh",
    "CacheRequestType.cc",
    "CacheRequestType.hh",
    "CacheResourceType.cc",
    "CacheResourceType.hh",
    "DirectoryRequestType.cc",
    "DirectoryRequestType.hh",
    "DMASequencerRequestType.cc",
    "DMASequencerRequestType.hh",
    "HtmCallbackMode.cc",
    "HtmCallbackMode.hh",
    "HtmFailedInCacheReason.cc",
    "HtmFailedInCacheReason.hh",
    "InvalidateGeneratorStatus.cc",
    "InvalidateGeneratorStatus.hh",
    "LinkDirection.cc",
    "LinkDirection.hh",
    "LockStatus.cc",
    "LockStatus.hh",
    "MachineType.cc",
    "MachineType.hh",
    "MaskPredictorIndex.cc",
    "MaskPredictorIndex.hh",
    "MaskPredictorTraining.cc",
    "MaskPredictorTraining.hh",
    "MaskPredictorType.cc",
    "MaskPredictorType.hh",
    "MemoryControlRequestType.cc",
    "MemoryControlRequestType.hh",
    "MemoryMsg.cc",
    "MemoryMsg.hh",
    "MemoryRequestType.cc",
    "MemoryRequestType.hh",
    "MessageSizeType.cc",
    "MessageSizeType.hh",
    "PrefetchBit.cc",
    "PrefetchBit.hh",
    "RequestStatus.cc",
    "RequestStatus.hh",
    "RubyAccessMode.cc",
    "RubyAccessMode.hh",
    "RubyRequestType.cc",
    "RubyRequestType.hh",
    "SequencerMsg.cc",
    "SequencerRequestType.cc",
    "SequencerRequestType.hh",
    "SequencerStatus.cc",
    "SequencerStatus.hh",
    "SeriesRequestGeneratorStatus.cc",
    "SeriesRequestGeneratorStatus.hh",
    "TesterStatus.cc",
    "TesterStatus.hh",
    "TransitionResult.cc",
    "TransitionResult.hh",
    "SequencerMsg.hh",
]

# These are headers that are created by slicc when processing the shared RubySlicc_interfaces.slicc
# These headers are used elsewhere in Ruby.
exported_headers = [
    {
        "header": "AccessPermission.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "AccessType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "CacheRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "CacheResourceType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "DirectoryRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "DMASequencerRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "HtmCallbackMode.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "HtmFailedInCacheReason.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "LinkDirection.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "MachineType.hh",
        "deps": [
            "//src/base:logging",
            "//src/mem/ruby/common:Address_hh",
            "//src/mem/ruby/common:TypeDefines_hh",
        ],
    },
    {
        "header": "MemoryMsg.hh",
        "deps": [
            "//src/mem/ruby/system:RubySystem_hh",
            "//src/mem/ruby/slicc_interface:RubySlicc_Util",
            ":MemoryRequestType_hh",
            ":MachineID_hh",
            ":DataBlock_hh",
            ":MessageSizeType_hh",
            ":PrefetchBit_hh",
            ":Message_hh",
        ],
    },
    {
        "header": "MemoryRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "MessageSizeType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "PrefetchBit.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "RequestStatus.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "RubyAccessMode.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "RubyRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "SequencerMsg.hh",
        "deps": [
            ":RubyAccessMode_hh",
            "//src/mem/ruby/protocol:SequencerRequestType_hh",
            "//src/mem/ruby/protocol:WriteMask_hh",
            "//src/mem/ruby/protocol:DataBlock_hh",
            "//src/mem/ruby/protocol:MessageSizeType_hh",
            "//src/mem/ruby/protocol:PrefetchBit_hh",
            "//src/mem/ruby/protocol:Message_hh",
            "//src/mem/ruby/slicc_interface:RubySlicc_Util",
            "//src/mem/ruby/system:RubySystem_hh",
        ],
    },
    {
        "header": "SequencerRequestType.hh",
        "deps": ["//src/base:logging"],
    },
    {
        "header": "TesterStatus.hh",
        "deps": ["//src/base:logging"],
    },
]

# These are headers outside of the protocol directory that are used by the generated headers.
# SLICC ends up creating new headers that reference these.
imported_headers = [
    "//src/mem/ruby/slicc_interface:AbstractCacheEntry",
    "//src/mem/ruby/slicc_interface:Message",
    "//src/mem/ruby/slicc_interface:RubyRequest",
    "//src/mem/ruby/common:Address",
    "//src/mem/ruby/common:BoolVec",
    "//src/mem/ruby/common:DataBlock",
    "//src/mem/ruby/common:ExpectedMap",
    "//src/mem/ruby/common:IntVec",
    "//src/mem/ruby/common:MachineID",
    "//src/mem/ruby/common:NetDest",
    "//src/mem/ruby/common:TriggerQueue",
    "//src/mem/ruby/common:Set",
    "//src/mem/ruby/common:WriteMask",
    "//src/mem/ruby/network:MessageBuffer",
    "//src/mem/ruby/structures:CacheMemory",
    "//src/mem/ruby/structures:DirectoryMemory",
    "//src/mem/ruby/structures:PerfectCacheMemory",
    "//src/mem/ruby/structures:PersistentTable",
    "//src/mem/ruby/structures:RubyPrefetcher",
    "//src/mem/ruby/structures:RubyPrefetcherProxy",
    "//src/mem/ruby/structures:TBEStorage",
    "//src/mem/ruby/structures:MN_TBEStorage",
    "//src/mem/ruby/structures:MN_TBETable",
    "//src/mem/ruby/structures:TBETable",
    "//src/mem/ruby/structures:TimerTable",
    "//src/mem/ruby/structures:WireBuffer",
    "//src/mem/ruby/system:DMASequencer",
    "//src/mem/ruby/system:Sequencer",
    # "//src/mem/ruby/system:GPUCoalescer",
    "//src/mem/ruby/system:HTMSequencer",
    # "//src/mem/ruby/system:VIPERCoalescer",
    # "//src/mem/ruby/system:VIPERSequencer",
]

filegroup(
    name = "shared_slicc",
    srcs = ["RubySlicc_interfaces.slicc"],
)

filegroup(
    name = "shared_sm_files",
    srcs = [
        "RubySlicc_ComponentMapping.sm",
        "RubySlicc_Defines.sm",
        "RubySlicc_Exports.sm",
        "RubySlicc_MemControl.sm",
        "RubySlicc_Types.sm",
        "RubySlicc_Util.sm",
    ],
)

# Nothing in Ruby should depend on this target directly. Instead, it should depend on something in
# exported_headers. The protocols will depend on this target directly.
generate_shared_slicc(
    name = "shared",
    exported_headers = exported_headers,
    protocol_outs = shared_outs,
    slicc_files = [":shared_slicc"],
    sm_srcs = [":shared_sm_files"],
    deps = [
        "//src/base:logging",
        "//src/mem/ruby/common",
        "//src/mem/ruby/slicc_interface:RubySlicc_Util",
        "//src/mem/ruby/structures:RubyPrefetcherProxy",
        "//src/mem/ruby/structures:TimerTable",
        "//src/mem/ruby/system:RubySystem_hh",
    ] + [":%s_hh" % header.split(":")[1] for header in imported_headers],
)

generate_imported_headers(
    name = "imported_headers",
    imported_headers = imported_headers,
)

# --------------------------------------------
# MESI_Two_Level protocol
# --------------------------------------------

gem5_slicc_cc_library(
    name = "MESI_Two_Level",
    cc_deps = default_protocol_cc_deps,
    protocol_outs = [
        "CoherenceRequestType.cc",
        "CoherenceRequestType.hh",
        "CoherenceResponseType.cc",
        "CoherenceResponseType.hh",
        "Directory_Controller.cc",
        "Directory_Controller.hh",
        "Directory_Entry.cc",
        "Directory_Entry.hh",
        "Directory_Event.cc",
        "Directory_Event.hh",
        "Directory_State.cc",
        "Directory_State.hh",
        "Directory_TBE.cc",
        "Directory_TBE.hh",
        "Directory_Transitions.cc",
        "Directory_Wakeup.cc",
        "DMA_Controller.cc",
        "DMA_Controller.hh",
        "DMA_Event.cc",
        "DMA_Event.hh",
        "DMA_State.cc",
        "DMA_State.hh",
        "DMA_TBE.cc",
        "DMA_TBE.hh",
        "DMA_Transitions.cc",
        "DMA_Wakeup.cc",
        "L1Cache_Controller.cc",
        "L1Cache_Controller.hh",
        "L1Cache_Entry.cc",
        "L1Cache_Entry.hh",
        "L1Cache_Event.cc",
        "L1Cache_Event.hh",
        "L1Cache_State.cc",
        "L1Cache_State.hh",
        "L1Cache_TBE.cc",
        "L1Cache_TBE.hh",
        "L1Cache_Transitions.cc",
        "L1Cache_Wakeup.cc",
        "L2Cache_Controller.cc",
        "L2Cache_Controller.hh",
        "L2Cache_Entry.cc",
        "L2Cache_Entry.hh",
        "L2Cache_Event.cc",
        "L2Cache_Event.hh",
        "L2Cache_State.cc",
        "L2Cache_State.hh",
        "L2Cache_TBE.cc",
        "L2Cache_TBE.hh",
        "L2Cache_Transitions.cc",
        "L2Cache_Wakeup.cc",
        "MESI_Two_Level_Directory_Controller.py",
        "MESI_Two_Level_DMA_Controller.py",
        "MESI_Two_Level_L1Cache_Controller.py",
        "MESI_Two_Level_L2Cache_Controller.py",
        "MESI_Two_LevelProtocolInfo.hh",
        "RequestMsg.cc",
        "RequestMsg.hh",
        "ResponseMsg.cc",
        "ResponseMsg.hh",
        "Types.hh",
    ],
    protocol_path = "MESI_Two_Level.slicc",
    sim_object_param_deps = default_sim_object_param_deps,
    sim_object_py_deps = default_sim_object_py_deps,
    slicc_includes = ["RubySlicc_interfaces.slicc"],
    # All .sm files mentioned in the slicc_includes
    sm_srcs = [
        "MESI_Two_Level-L1cache.sm",
        "MESI_Two_Level-L2cache.sm",
        "MESI_Two_Level-dir.sm",
        "MESI_Two_Level-dma.sm",
        "MESI_Two_Level-msg.sm",
    ] + [":shared_sm_files"],
)
# --------------------------------------------
# CHI protocol
# --------------------------------------------

py_library(
    name = "chi_init",
    srcs = ["CHI/__init__.py"],
    deps = [":protocol_init"],
)

gem5_slicc_cc_library(
    name = "CHI",
    cc_deps = default_protocol_cc_deps + [
        "//src/mem:debug_LLSC",
        "//src/mem/ruby:debug_RubyProtocol",
    ],
    cc_deps_no_header = [
        "//src/mem/ruby/structures:MN_TBETable",
    ],
    exported_headers = [
        {
            "header": "MiscNode_TBE.hh",
            "deps": [
                ":MachineID_hh",
                ":NetDest_hh",
                ":TriggerQueue_hh",
                ":ExpectedMap_hh",
                "//src/mem/ruby/slicc_interface:RubySlicc_Util",
                ":CHIRequestType_hh",
                ":CHIResponseType_hh",
                ":CHIDataType_hh",
                ":MiscNode_State_hh",
                ":MiscNode_Event_hh",
            ],
        },
        {"header": "CHIRequestType.hh", "deps": []},
        {"header": "CHIResponseType.hh", "deps": []},
        {"header": "CHIDataType.hh", "deps": []},
        {
            "header": "MiscNode_State.hh",
            "deps": [":AccessPermission_hh"],
        },
        {"header": "MiscNode_Event.hh", "deps": []},
        {
            "header": "CHIProtocolInfo.hh",
            "deps": ["//src/mem/ruby/slicc_interface:ProtocolInfo"],
        },
        {
            "header": "CHIDataMsg.hh",
            "deps": [
                ":CHIDataType_hh",
                ":MachineID_hh",
                ":NetDest_hh",
                ":DataBlock_hh",
                ":WriteMask_hh",
                ":MessageSizeType_hh",
                ":Message_hh",
                "//src/mem/ruby/slicc_interface:RubySlicc_Util",
            ],
        },
        {
            "header": "CHIRequestMsg.hh",
            "deps": [
                ":CHIRequestType_hh",
                ":MachineID_hh",
                ":NetDest_hh",
                ":DataBlock_hh",
                ":WriteMask_hh",
                ":MessageSizeType_hh",
                ":Message_hh",
                "//src/mem/ruby/slicc_interface:RubySlicc_Util",
            ],
        },
        {
            "header": "CHIResponseMsg.hh",
            "deps": [
                ":CHIResponseType_hh",
                ":DataBlock_hh",
                ":MachineID_hh",
                ":NetDest_hh",
                ":WriteMask_hh",
                ":MessageSizeType_hh",
                ":Message_hh",
                "//src/mem/ruby/slicc_interface:RubySlicc_Util",
            ],
        },
    ],
    protocol_outs = [
        "Cache_CacheEntry.cc", "Cache_CacheEntry.hh",
        "Cache_Controller.cc", "Cache_Controller.hh",
        "Cache_DirEntry.cc", "Cache_DirEntry.hh",
        "Cache_Event.cc", "Cache_Event.hh",
        "Cache_ReplacementMsg.cc", "Cache_ReplacementMsg.hh",
        "Cache_RequestType.cc", "Cache_RequestType.hh",
        "Cache_RetryQueueEntry.cc", "Cache_RetryQueueEntry.hh",
        "Cache_RetryTriggerMsg.cc", "Cache_RetryTriggerMsg.hh",
        "Cache_State.cc", "Cache_State.hh",
        "Cache_TBE.cc", "Cache_TBE.hh",
        "Cache_Transitions.cc",
        "Cache_TriggerMsg.cc", "Cache_TriggerMsg.hh",
        "Cache_Wakeup.cc",
        "CHI_Cache_Controller.py",
        "CHI_Memory_Controller.py",
        "CHI_MiscNode_Controller.py",
        "CHIDataMsg.cc", "CHIDataMsg.hh",
        "CHIDataType.cc", "CHIDataType.hh",
        "CHIProtocolInfo.hh",
        "CHIRequestMsg.cc", "CHIRequestMsg.hh",
        "CHIRequestType.cc", "CHIRequestType.hh",
        "CHIResponseMsg.cc", "CHIResponseMsg.hh",
        "CHIResponseType.cc", "CHIResponseType.hh",
        "Memory_Controller.cc", "Memory_Controller.hh",
        "Memory_Event.cc", "Memory_Event.hh",
        "Memory_RetryQueueEntry.cc", "Memory_RetryQueueEntry.hh",
        "Memory_State.cc", "Memory_State.hh",
        "Memory_TBE.cc", "Memory_TBE.hh",
        "Memory_Transitions.cc",
        "Memory_TriggerMsg.cc", "Memory_TriggerMsg.hh",
        "Memory_Wakeup.cc",
        "MiscNode_Controller.cc", "MiscNode_Controller.hh",
        "MiscNode_Event.cc", "MiscNode_Event.hh",
        "MiscNode_RetryQueueEntry.cc", "MiscNode_RetryQueueEntry.hh",
        "MiscNode_RetryTriggerMsg.cc", "MiscNode_RetryTriggerMsg.hh",
        "MiscNode_State.cc", "MiscNode_State.hh",
        "MiscNode_TBE.cc", "MiscNode_TBE.hh",
        "MiscNode_Transitions.cc",
        "MiscNode_TriggerMsg.cc", "MiscNode_TriggerMsg.hh",
        "MiscNode_Wakeup.cc",
        "Types.hh",
    ],
    protocol_path = "CHI/CHI.slicc",
    sim_object_param_deps = default_sim_object_param_deps,
    sim_object_py_deps = default_sim_object_py_deps + [":chi_init"],
    slicc_includes = [":shared_slicc"],
    sm_srcs = [
        "CHI/CHI-cache-actions.sm",
        "CHI/CHI-cache-funcs.sm",
        "CHI/CHI-cache-ports.sm",
        "CHI/CHI-cache-transitions.sm",
        "CHI/CHI-cache.sm",
        "CHI/CHI-dvm-misc-node-actions.sm",
        "CHI/CHI-dvm-misc-node-funcs.sm",
        "CHI/CHI-dvm-misc-node-ports.sm",
        "CHI/CHI-dvm-misc-node-transitions.sm",
        "CHI/CHI-dvm-misc-node.sm",
        "CHI/CHI-mem.sm",
        "CHI/CHI-msg.sm",
    ] + [":shared_sm_files"],
)
