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
    else:
        new_local_repository(
            name = "gem5-raw",
            path = effective.source_root,
            build_file_content = "# Raw gem5 source tree.\n",
        )

    # Create @gem5_sources via the overlay repository rule.
    gem5_configure(
        name = effective.name,
        gem5_raw = "@gem5-raw//:CMakeLists.txt",
        overlay_path = "//:gem5-overlay/.bazelignore",
    )

    # Create ext/ library repositories.
    # In remote mode, use repository rules that resolve @gem5-raw at
    # repository-rule time (avoids module_ctx.path() circular dependency).
    # In local mode, use new_local_repository with direct filesystem paths.
    for mod in module_ctx.modules:
        for _tag in mod.tags.ext_libraries:
            if effective.is_remote:
                _create_ext_libraries_remote()
            else:
                _create_ext_libraries_local(effective.source_root)

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

# Ext library definitions: (repo_name, ext_subdir, build_file)
_EXT_LIBRARIES = [
    ("gem5_ext_libelf", "libelf", "libelf.BUILD"),
    ("gem5_ext_libfdt", "libfdt", "libfdt.BUILD"),
    ("gem5_ext_fputils", "fputils", "fputils.BUILD"),
    ("gem5_ext_softfloat", "softfloat", "softfloat.BUILD"),
    ("gem5_ext_dsent", "dsent", "dsent.BUILD"),
    ("gem5_ext_drampower", "drampower", "drampower.BUILD"),
    ("gem5_ext_dramsim2", "dramsim2", "dramsim2.BUILD"),
    ("gem5_ext_dramsim3", "dramsim3", "dramsim3.BUILD"),
    ("gem5_ext_dramsys", "dramsys", "dramsys.BUILD"),
    ("gem5_ext_nomali", "nomali", "nomali.BUILD"),
    ("gem5_ext_iostream3", "iostream3", "iostream3.BUILD"),
    ("gem5_ext_magic_enum", "magic_enum", "magic_enum.BUILD"),
    ("gem5_ext_systemc", "systemc", "systemc.BUILD"),
    ("gem5_ext_pybind11", "pybind11", "pybind11.BUILD"),
    ("gem5_ext_ply", "ply", "ply.BUILD"),
    ("gem5_ext_dnet", "dnet", "dnet.BUILD"),
    ("gem5_ext_googletest", "googletest", "googletest.BUILD"),
    ("gem5_ext_gdbremote", "gdbremote", "gdbremote.BUILD"),
    ("gem5_ext_x11keysym", "x11keysym", "x11keysym.BUILD"),
]

def _create_ext_libraries_local(source_root):
    """Create ext/ library repos using local filesystem paths."""
    ext_path = source_root + "/ext"
    for repo_name, subdir, build_file in _EXT_LIBRARIES:
        new_local_repository(
            name = repo_name,
            path = ext_path + "/" + subdir,
            build_file = "//third_party_build:" + build_file,
        )

def _create_ext_libraries_remote():
    """Create ext/ library repos that resolve @gem5-raw at repository-rule time.

    In remote mode, module_ctx.path() on @gem5-raw causes a circular
    dependency (the extension creates gem5-raw and then tries to read
    from it). Instead, each ext repo uses a repository rule that
    resolves @gem5-raw via repository_ctx.path(), which works because
    repository rules can access other repos without cycles.
    """
    for repo_name, subdir, build_file in _EXT_LIBRARIES:
        _gem5_ext_repo(
            name = repo_name,
            raw_root_marker = "@gem5-raw//:BUILD.bazel",
            subdir = subdir,
            build_file = "//third_party_build:" + build_file,
        )

def _gem5_ext_repo_impl(repository_ctx):
    """Repository rule that creates an ext library from @gem5-raw."""
    raw_root = repository_ctx.path(repository_ctx.attr.raw_root_marker).dirname
    ext_dir = raw_root.get_child("ext").get_child(repository_ctx.attr.subdir)
    for entry in ext_dir.readdir():
        repository_ctx.symlink(entry, entry.basename)
    repository_ctx.symlink(
        repository_ctx.path(repository_ctx.attr.build_file),
        "BUILD.bazel",
    )

_gem5_ext_repo = repository_rule(
    implementation = _gem5_ext_repo_impl,
    attrs = {
        "raw_root_marker": attr.label(mandatory = True),
        "subdir": attr.string(mandatory = True),
        "build_file": attr.label(mandatory = True),
    },
)

gem5_repos_extension = module_extension(
    implementation = _gem5_repos_extension_impl,
    tag_classes = {
        "configure": _configure_tag,
        "system_python": _system_python_tag,
        "ext_libraries": _ext_libraries_tag,
    },
)
