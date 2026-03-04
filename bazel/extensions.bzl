"""Module extensions for the gem5 Bazel build.

Provides three tag classes:
- configure: Creates the @gem5_sources overlay repository and @gem5-raw source repo
- system_python: Detects system Python headers
- ext_libraries: Sets up ext/ library repositories

Source root resolution (in priority order):
1. Explicit source_root on ANY configure tag (local override / custom layouts)
2. source_urls on ANY configure tag (archive / remote mode -- fetch via http_archive)
3. Auto-detect: parent of the gem5 module's workspace root (standalone / local_path_override)

Multiple configure tags are merged into one effective configuration. Tags with
explicit source_root or source_urls take precedence over empty tags. At most one
@gem5-raw repository is ever created.
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

    When used via local_path_override, the external cache entry is a
    symlink to the real filesystem path. We follow symlinks via
    .realpath so that the auto-detected parent directory points to
    the actual gem5 source tree, not the Bazel cache.
    """
    module_bzl = module_ctx.path(Label("//:MODULE.bazel"))
    return str(module_bzl.realpath.dirname)

def _resolve_effective_configure(module_ctx):
    """Merge all configure tags into one effective configuration.

    Multiple modules may provide configure tags (gem5's own MODULE.bazel
    plus downstream MODULE.bazel). Tags with explicit source_root or
    source_urls take precedence over empty/default tags. The first
    non-empty source specification wins.

    Returns: struct(name, source_root, source_urls, source_strip_prefix,
                    source_integrity, is_remote)
    """
    name = ""
    source_root = ""
    source_urls = []
    strip_prefix = ""
    integrity = ""

    for mod in module_ctx.modules:
        for tag in mod.tags.configure:
            if not name:
                name = tag.name
            if tag.source_root and not source_root:
                source_root = tag.source_root
            if tag.source_urls and not source_urls:
                source_urls = list(tag.source_urls)
                strip_prefix = tag.source_strip_prefix
                integrity = tag.source_integrity

    # Priority: explicit source_root > source_urls > auto-detect
    is_remote = len(source_urls) > 0 and not source_root

    if not is_remote and not source_root:
        gem5_root = _find_gem5_module_root(module_ctx)
        # Check parent directory first (local_path_override: bazel/ is a
        # subdirectory of the gem5 source tree), then the module root itself
        # (archive_override: source is co-located with MODULE.bazel).
        parent = gem5_root + "/.."
        if module_ctx.path(parent + "/src").exists:
            source_root = parent
        elif module_ctx.path(gem5_root + "/src").exists:
            source_root = gem5_root
        else:
            source_root = parent

    return struct(
        name = name if name else "gem5_sources",
        source_root = source_root,
        source_urls = source_urls,
        source_strip_prefix = strip_prefix,
        source_integrity = integrity,
        is_remote = is_remote,
    )

def _gem5_repos_extension_impl(module_ctx):
    """Module extension that creates gem5 repositories.

    Creates exactly one @gem5-raw repo and one @gem5_sources overlay repo,
    then sets up ext/ libraries and system Python detection.
    """
    effective = _resolve_effective_configure(module_ctx)

    # Create @gem5-raw exactly once.
    if effective.is_remote:
        http_archive(
            name = "gem5-raw",
            urls = effective.source_urls,
            strip_prefix = effective.source_strip_prefix,
            integrity = effective.source_integrity if effective.source_integrity else "",
            build_file_content = "# Raw gem5 source tree.\n",
        )

        # Resolve the fetched archive path for ext repo creation.
        # module_ctx.path() triggers the fetch and returns the local path.
        source_root = str(module_ctx.path(Label("@gem5-raw//:BUILD.bazel")).dirname)
    else:
        new_local_repository(
            name = "gem5-raw",
            path = effective.source_root,
            build_file_content = "# Raw gem5 source tree.\n",
        )
        source_root = effective.source_root

    # Create @gem5_sources via the overlay repository rule.
    gem5_configure(
        name = effective.name,
        gem5_raw = "@gem5-raw//:CMakeLists.txt",
        overlay_path = "//:gem5-overlay/.bazelignore",
    )

    # Create ext/ library repositories using the resolved source root.
    # This works in both local mode (filesystem path) and remote mode
    # (path resolved from the fetched @gem5-raw archive).
    for mod in module_ctx.modules:
        for _tag in mod.tags.ext_libraries:
            _create_ext_libraries(source_root)

    # System Python detection.
    for mod in module_ctx.modules:
        for tag in mod.tags.system_python:
            system_python(name = tag.name)

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
    _ext_local_repo("gem5_ext_gdbremote", ext_path + "/gdbremote", "gdbremote.BUILD")
    _ext_local_repo("gem5_ext_x11keysym", ext_path + "/x11keysym", "x11keysym.BUILD")

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
