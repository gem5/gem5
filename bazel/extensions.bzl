"""Module extensions for the gem5 Bazel build.

Provides three tag classes:
- configure: Creates the @gem5_sources overlay repository and @gem5-raw source repo
- system_python: Detects system Python headers
- ext_libraries: Sets up ext/ library repositories

Source root detection (in priority order):
1. Explicit source_root on the configure tag (for archive or custom layouts)
2. Auto-detect: parent of the gem5 module's workspace root (for standalone
   and local_path_override modes where bazel/ is inside a gem5 source tree)
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")
load("//:configure.bzl", "gem5_configure")
load("//:python_detect.bzl", "system_python")

def _find_gem5_module_root(module_ctx):
    """Find the gem5 Bazel module's workspace directory.

    Uses module_ctx.path() to resolve a label in the gem5 module, which
    gives us an absolute filesystem path. The gem5 module IS the module
    that defines this extension, so //:MODULE.bazel resolves to it.

    In standalone mode, this is the bazel/ directory inside the gem5 tree.
    In downstream local_path_override mode, this is wherever the override
    points (e.g., third_party/gem5/bazel).
    """
    module_bzl = module_ctx.path(Label("//:MODULE.bazel"))
    return str(module_bzl.dirname)

def _resolve_source_root(module_ctx, configure_tags):
    """Determine the gem5 source root from configure tags and module graph.

    Returns: (source_root_path, is_remote) tuple.
      - source_root_path: filesystem path to the gem5 source root, or None
        if remote fetch is needed.
      - is_remote: True when the source must be fetched via http_archive.
    """
    # Check if any configure tag provides an explicit source_root
    for tag in configure_tags:
        if tag.source_root:
            return (tag.source_root, False)

    # Check if any configure tag requests a remote source fetch
    for tag in configure_tags:
        if tag.source_urls:
            return (None, True)

    # Auto-detect: parent of the gem5 module's workspace root.
    # This works for standalone mode (bazel/ inside gem5 tree) and
    # local_path_override mode (points to third_party/gem5/bazel).
    gem5_root = _find_gem5_module_root(module_ctx)
    return (gem5_root + "/..", False)

def _gem5_repos_extension_impl(module_ctx):
    """Module extension that creates gem5 repositories."""

    # Collect all configure tags across modules (gem5's own + downstream)
    all_configure_tags = []
    for mod in module_ctx.modules:
        all_configure_tags.extend(mod.tags.configure)

    source_root, is_remote = _resolve_source_root(module_ctx, all_configure_tags)

    for mod in module_ctx.modules:
        for tag in mod.tags.configure:
            if is_remote:
                # Remote mode: fetch the full gem5 source as @gem5-raw
                # via http_archive. The URLs should point to the full
                # gem5 archive (not the bazel/-stripped version).
                http_archive(
                    name = "gem5-raw",
                    urls = tag.source_urls,
                    strip_prefix = tag.source_strip_prefix,
                    integrity = tag.source_integrity if tag.source_integrity else "",
                    build_file_content = "# Raw gem5 source tree.\n",
                )
            else:
                # Local mode: symlink to the source root on disk.
                new_local_repository(
                    name = "gem5-raw",
                    path = source_root,
                    build_file_content = "# Raw gem5 source tree.\n",
                )

            # Create @gem5_sources via the overlay repository rule.
            gem5_configure(
                name = tag.name,
                gem5_raw = "@gem5-raw//:CMakeLists.txt",
                overlay_path = "//:gem5-overlay/.bazelignore",
            )

        for tag in mod.tags.system_python:
            system_python(name = tag.name)

        for tag in mod.tags.ext_libraries:
            _create_ext_libraries(source_root)

_configure_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
        "source_root": attr.string(
            default = "",
            doc = "Explicit gem5 source root path (filesystem). " +
                  "Use for archive_override when auto-detection fails.",
        ),
        "source_urls": attr.string_list(
            default = [],
            doc = "URLs to fetch the full gem5 source archive. " +
                  "Used in archive/remote mode to create @gem5-raw.",
        ),
        "source_strip_prefix": attr.string(
            default = "",
            doc = "strip_prefix for the source archive (e.g. 'gem5-{commit}').",
        ),
        "source_integrity": attr.string(
            default = "",
            doc = "SRI hash for the source archive.",
        ),
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

def _create_ext_libraries(source_root):
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
    _ext_local_repo("gem5_ext_googletest", ext_path + "/googletest", "googletest.BUILD")

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
