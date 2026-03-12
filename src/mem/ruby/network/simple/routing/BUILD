load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "BaseRoutingUnit",
    hdrs = ["BaseRoutingUnit.hh"],
    deps = [
        "//src/mem/ruby/network:Network",
        "//src/mem/ruby/network/simple:base_routing_unit_sim_object",
        "//src/mem/ruby/slicc_interface:Message",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "WeightBased",
    srcs = ["WeightBased.cc"],
    hdrs = ["WeightBased.hh"],
    deps = [
        ":BaseRoutingUnit",
        "//src/base:random",
        "//src/mem/ruby/network/simple:Switch",
        "//src/mem/ruby/network/simple:weight_based_sim_object",
    ],
)
