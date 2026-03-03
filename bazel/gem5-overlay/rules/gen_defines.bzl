"""Rule for generating m5/defines.py containing buildEnv dictionary.

Reads build flag values and generates a Python file with the buildEnv
dictionary, then embeds it as C++ via marshal.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _gen_defines_py_impl(ctx):
    """Generate defines.py from build flags."""
    out = ctx.actions.declare_file("m5/defines.py")

    # Build the defines content from flag values
    lines = [
        "buildEnv = {",
        "    'BUILD_ISA': '{}',".format(ctx.attr.build_isa),
        "    'USE_X86_ISA': {},".format(ctx.attr.use_x86_isa),
        "    'USE_ARM_ISA': {},".format(ctx.attr.use_arm_isa),
        "    'USE_RISCV_ISA': {},".format(ctx.attr.use_riscv_isa),
        "    'USE_MIPS_ISA': {},".format(ctx.attr.use_mips_isa),
        "    'USE_POWER_ISA': {},".format(ctx.attr.use_power_isa),
        "    'USE_SPARC_ISA': {},".format(ctx.attr.use_sparc_isa),
        "    'BUILD_GPU': {},".format(ctx.attr.build_gpu),
        "    'TARGET_GPU_ISA': '{}',".format(ctx.attr.gpu_isa),
        "    'RUBY': {},".format(ctx.attr.use_ruby),
        "    'PROTOCOL': '{}',".format(ctx.attr.ruby_protocol),
        "    'USE_SYSTEMC': {},".format(ctx.attr.use_systemc),
        "    'USE_KVM': {},".format(ctx.attr.use_kvm),
        "    'HAVE_PROTOBUF': {},".format(ctx.attr.have_protobuf),
        "    'HAVE_PNG': {},".format(ctx.attr.have_png),
        "    'HAVE_HDF5': {},".format(ctx.attr.have_hdf5),
        "}",
    ]

    ctx.actions.write(out, "\n".join(lines) + "\n")
    return [DefaultInfo(files = depset([out]))]

_gen_defines_py = rule(
    implementation = _gen_defines_py_impl,
    attrs = {
        "build_isa": attr.string(default = "x86"),
        "use_x86_isa": attr.bool(default = True),
        "use_arm_isa": attr.bool(default = False),
        "use_riscv_isa": attr.bool(default = False),
        "use_mips_isa": attr.bool(default = False),
        "use_power_isa": attr.bool(default = False),
        "use_sparc_isa": attr.bool(default = False),
        "build_gpu": attr.bool(default = False),
        "gpu_isa": attr.string(default = "vega"),
        "use_ruby": attr.bool(default = True),
        "ruby_protocol": attr.string(default = "MESI_Two_Level"),
        "use_systemc": attr.bool(default = False),
        "use_kvm": attr.bool(default = False),
        "have_protobuf": attr.bool(default = False),
        "have_png": attr.bool(default = False),
        "have_hdf5": attr.bool(default = False),
    },
)

def gem5_gen_defines(name, visibility = None, **kwargs):
    """Generate m5/defines.py with buildEnv dictionary.

    Args:
        name: Target name.
        visibility: Visibility.
        **kwargs: Flag values to set in buildEnv.
    """
    _gen_defines_py(
        name = name,
        visibility = visibility,
        **kwargs
    )
