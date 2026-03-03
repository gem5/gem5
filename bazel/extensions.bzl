"""Module extensions for the gem5 Bazel build.

Provides three tag classes:
- configure: Creates the @gem5 overlay repository
- system_python: Detects system Python headers
- ext_libraries: Sets up ext/ library repositories
"""

load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")
load("//:configure.bzl", "gem5_configure")
load("//:python_detect.bzl", "system_python")

def _find_gem5_source_root(module_ctx):
    """Detect the gem5 source root.

    In standalone mode (bazel/ is inside a gem5 source tree), the source root
    is the parent directory of bazel/. This is detected by checking if the
    current module is root and its parent contains src/base/version.cc.
    """
    # The module root is the bazel/ directory. Parent is gem5 source root.
    # We use the module's path to find the parent.
    return "../"

def _gem5_repos_extension_impl(module_ctx):
    """Module extension that creates gem5 repositories."""
    source_root = _find_gem5_source_root(module_ctx)

    for mod in module_ctx.modules:
        for tag in mod.tags.configure:
            # Create @gem5-raw pointing to the gem5 source root.
            new_local_repository(
                name = "gem5-raw",
                path = source_root,
                build_file_content = "# Raw gem5 source tree. BUILD files come from the overlay.\n",
            )

            # Create @gem5 via the overlay repository rule.
            gem5_configure(
                name = tag.name,
                gem5_raw = "@gem5-raw//:CMakeLists.txt",
                overlay_path = "//:gem5-overlay/.bazelignore",
            )

        for tag in mod.tags.system_python:
            system_python(name = tag.name)

        for tag in mod.tags.ext_libraries:
            _create_ext_libraries(module_ctx, source_root)

_configure_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
    },
)

_system_python_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
    },
)

_ext_libraries_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
    },
)

def _create_ext_libraries(module_ctx, source_root):
    """Create repositories for each ext/ library."""
    ext_path = source_root + "/ext"

    _ext_local_repo("gem5_ext_libelf", ext_path + "/libelf", "libelf.BUILD")
    _ext_local_repo("gem5_ext_libfdt", ext_path + "/libfdt", "libfdt.BUILD")
    _ext_local_repo("gem5_ext_fputils", ext_path + "/fputils", "fputils.BUILD")
    _ext_local_repo("gem5_ext_softfloat", ext_path + "/softfloat", "softfloat.BUILD")
    _ext_local_repo("gem5_ext_dsent", ext_path + "/dsent", "dsent.BUILD")
    _ext_local_repo("gem5_ext_drampower", ext_path + "/drampower", "drampower.BUILD")
    _ext_local_repo("gem5_ext_dramsim2", ext_path + "/dramsim2", "dramsim2.BUILD")
    _ext_local_repo("gem5_ext_dramsim3", ext_path + "/dramsim3", "dramsim3.BUILD")
    _ext_local_repo("gem5_ext_dramsys", ext_path + "/dramsys", "dramsys.BUILD")
    _ext_local_repo("gem5_ext_nomali", ext_path + "/nomali", "nomali.BUILD")
    _ext_local_repo("gem5_ext_iostream3", ext_path + "/iostream3", "iostream3.BUILD")
    _ext_local_repo("gem5_ext_magic_enum", ext_path + "/magic_enum", "magic_enum.BUILD")
    _ext_local_repo("gem5_ext_systemc", ext_path + "/systemc", "systemc.BUILD")
    _ext_local_repo("gem5_ext_pybind11", ext_path + "/pybind11", "pybind11.BUILD")
    _ext_local_repo("gem5_ext_ply", ext_path + "/ply", "ply.BUILD")
    _ext_local_repo("gem5_ext_dnet", ext_path + "/dnet", "dnet.BUILD")

def _ext_local_repo(name, path, build_file):
    """Create a new_local_repository for an ext/ library."""
    new_local_repository(
        name = name,
        path = path,
        build_file = "//third_party_build:" + build_file,
    )

gem5_repos_extension = module_extension(
    implementation = _gem5_repos_extension_impl,
    tag_classes = {
        "configure": _configure_tag,
        "system_python": _system_python_tag,
        "ext_libraries": _ext_libraries_tag,
    },
)
