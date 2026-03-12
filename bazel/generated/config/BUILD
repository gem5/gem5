load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "config",
    hdrs = [
        "build_gpu.hh",
        "have_deprecated_namespace.hh",
        "have_fenv.hh",
        "have_hdf5.hh",
        "have_png.hh",
        "have_protobuf.hh",
        "have_tuntap.hh",
        "have_valgrind.hh",
        "kvm_isa.hh",
        "use_kvm.hh",
        "use_posix_clock.hh",
    ],
    includes = [".."],
)

gem5_cc_library(
    name = "have_perf_attr_exclude_host",
    hdrs = ["have_perf_attr_exclude_host.hh"],
)
