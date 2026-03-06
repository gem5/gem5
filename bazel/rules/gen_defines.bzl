"""Rule for generating m5/defines.py containing buildEnv dictionary.

Generates a Python file with the buildEnv dictionary from build flag values.
All flag attributes are strings so they can accept select() in BUILD files.
The generated file is then embedded as C++ via the py_source marshal pipeline.
"""

def _gen_defines_py_impl(ctx):
    """Generate defines.py from build flags passed as string attrs."""
    out = ctx.actions.declare_file("m5/defines.py")

    lines = [
        "buildEnv = {",
        "    'BUILD_ISA': '{}',".format(ctx.attr.build_isa),
        "    'TARGET_ISA': '{}',".format(ctx.attr.build_isa),
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
        "    'HAVE_KVM': {},".format(ctx.attr.use_kvm),
        "    'HAVE_PROTOBUF': {},".format(ctx.attr.have_protobuf),
        "    'HAVE_PNG': {},".format(ctx.attr.have_png),
        "    'HAVE_HDF5': {},".format(ctx.attr.have_hdf5),
        "    'HAVE_TUNTAP': {},".format(ctx.attr.have_tuntap),
        "    'HAVE_CAPSTONE': {},".format(ctx.attr.have_capstone),
        "    'USE_CAPSTONE': {},".format(ctx.attr.have_capstone),
        "    'HAVE_FENV': {},".format(ctx.attr.have_fenv),
        "    'HAVE_POSIX_CLOCK': {},".format(ctx.attr.use_posix_clock),
        "    'USE_POSIX_CLOCK': {},".format(ctx.attr.use_posix_clock),
        "    'HAVE_VALGRIND': {},".format(ctx.attr.have_valgrind),
        "    'USE_TEST_OBJECTS': {},".format(ctx.attr.use_test_objects),
        "    'HAVE_SYSTEMC': {},".format(ctx.attr.use_systemc),
        "    'HAVE_DEPRECATED_NAMESPACE': True,",
        "    'BUILD_TLM': {},".format(ctx.attr.build_tlm),
        "    'KVM_ISA': '{}',".format(ctx.attr.kvm_isa),
        "    'VEGA_GPU_ISA': {},".format(ctx.attr.build_gpu),
        "    'NUMBER_BITS_PER_SET': {},".format(ctx.attr.number_bits_per_set),
        "    'SLICC_HTML': False,",
    ]

    # Per-protocol flags for gem5.runtime.get_supported_protocols()
    if ctx.attr.ruby_protocols:
        lines.append("    'USE_MULTIPLE_PROTOCOLS': True,")
        for proto in ctx.attr.ruby_protocols:
            lines.append("    'RUBY_PROTOCOL_{}': True,".format(proto))

    lines.append("}")

    ctx.actions.write(out, "\n".join(lines) + "\n")
    return [DefaultInfo(files = depset([out]))]

_gen_defines_py = rule(
    implementation = _gen_defines_py_impl,
    attrs = {
        "build_isa": attr.string(default = "x86"),
        "use_x86_isa": attr.string(default = "True"),
        "use_arm_isa": attr.string(default = "False"),
        "use_riscv_isa": attr.string(default = "False"),
        "use_mips_isa": attr.string(default = "False"),
        "use_power_isa": attr.string(default = "False"),
        "use_sparc_isa": attr.string(default = "False"),
        "build_gpu": attr.string(default = "False"),
        "gpu_isa": attr.string(default = "vega"),
        "use_ruby": attr.string(default = "True"),
        "ruby_protocol": attr.string(default = "MESI_Two_Level"),
        "ruby_protocols": attr.string_list(default = []),
        "use_systemc": attr.string(default = "False"),
        "use_kvm": attr.string(default = "False"),
        "have_protobuf": attr.string(default = "False"),
        "have_png": attr.string(default = "False"),
        "have_hdf5": attr.string(default = "False"),
        "have_tuntap": attr.string(default = "False"),
        "have_capstone": attr.string(default = "False"),
        "have_fenv": attr.string(default = "True"),
        "use_posix_clock": attr.string(default = "False"),
        "have_valgrind": attr.string(default = "False"),
        "use_test_objects": attr.string(default = "False"),
        "build_tlm": attr.string(default = "False"),
        "kvm_isa": attr.string(default = "x86"),
        "number_bits_per_set": attr.string(default = "64"),
    },
)

def gem5_gen_defines(name, visibility = None, **kwargs):
    """Generate m5/defines.py with buildEnv dictionary.

    All flag parameters accept select() for build-flag-dependent values.

    Args:
        name: Target name.
        visibility: Visibility.
        **kwargs: Flag values (as strings: "True"/"False") to set in buildEnv.
    """
    _gen_defines_py(
        name = name,
        visibility = visibility,
        **kwargs
    )
