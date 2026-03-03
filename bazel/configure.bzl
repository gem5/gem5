"""Repository rule that creates the @gem5 repository by overlaying BUILD files
onto the gem5 source tree.

The overlay mechanism follows LLVM's utils/bazel/ pattern:
- Symlinks source directories from @gem5-raw (the raw source tree)
- Overlays BUILD files from gem5-overlay/
- Generates static (flag-independent) configuration: version info, SLICC manifests

All flag-dependent artifacts (config headers, switching headers, defines.py)
are generated at build time by custom rules using select().
"""

MAX_TRAVERSAL_STEPS = 1000000

def _overlay_directories(repository_ctx, src_root, overlay_root):
    """Merge overlay BUILD files with source tree via symlinks.

    Algorithm (depth-first):
    For each directory in the overlay:
      1. Symlink all FILES from the overlay (BUILD.bazel, .bzl, etc.)
      2. Record overlay subdirectory names
      3. For source directory entries:
         - If overlay has a subdir with same name: skip (will recurse)
         - Otherwise: symlink entire entry (file or directory) from source
    This minimizes symlink count by symlinking entire source directories
    when the overlay has no overrides in that subtree.
    """
    target_root = repository_ctx.path(".")
    stack = ["."]

    for _ in range(MAX_TRAVERSAL_STEPS):
        if not stack:
            return
        rel_dir = stack.pop()

        overlay_entries = {}

        # Symlink overlay files; queue overlay dirs for future iterations.
        overlay_dir = overlay_root.get_child(rel_dir)
        if overlay_dir.exists:
            for entry in overlay_dir.readdir():
                name = entry.basename
                overlay_entries[name] = None
                full_rel_path = rel_dir + "/" + name
                if entry.is_dir:
                    stack.append(full_rel_path)
                else:
                    repository_ctx.symlink(
                        entry,
                        target_root.get_child(full_rel_path),
                    )

        # Symlink source entries not shadowed by overlay entries.
        src_dir = src_root.get_child(rel_dir)
        if src_dir.exists:
            for src_entry in src_dir.readdir():
                name = src_entry.basename
                if name in overlay_entries:
                    continue
                repository_ctx.symlink(
                    src_entry,
                    target_root.get_child(rel_dir + "/" + name),
                )

    fail("overlay_directories: exceeded MAX_TRAVERSAL_STEPS ({}). ".format(
        MAX_TRAVERSAL_STEPS,
    ) + "Tree too large or filesystem cycle?")

def _extract_version(repository_ctx, src_root):
    """Extract gem5 version from src/base/version.cc."""
    version_file = src_root.get_child("src/base/version.cc")
    content = repository_ctx.read(version_file)
    version = "unknown"
    for line in content.splitlines():
        if "gem5Version" in line:
            # Extract string between double quotes
            start = line.find('"')
            end = line.rfind('"')
            if start >= 0 and end > start:
                version = line[start + 1:end]
            break
    return version

def _generate_slicc_manifests(repository_ctx, src_root, module_root):
    """Run SLICC discovery for each protocol and write manifest .bzl file.

    Uses the slicc_discover.py tool from bazel/tools/ to parse each protocol's
    .slicc file and enumerate expected C++ output files. Results are written to
    configs/slicc_manifests.bzl for use by the gem5_slicc_protocol() rule.

    Falls back to empty manifests if discovery fails (slicc.bzl will use tree
    artifacts as fallback when a protocol's manifest is empty).
    """
    discover_script = module_root.get_child("tools/slicc_discover.py")

    if not discover_script.exists:
        # No discovery script available; write empty manifests.
        repository_ctx.file(
            "configs/slicc_manifests.bzl",
            content = '"""SLICC manifests (empty -- discovery unavailable)."""\n\n' +
                      "SLICC_MANIFESTS = {}\n",
        )
        return

    result = repository_ctx.execute(
        ["python3", str(discover_script), str(src_root)],
        timeout = 300,
        quiet = True,
    )

    if result.return_code == 0 and result.stdout.strip():
        repository_ctx.file(
            "configs/slicc_manifests.bzl",
            content = result.stdout,
        )
    else:
        # Discovery failed; write empty manifests so slicc.bzl can fall back.
        if result.stderr:
            # buildifier: disable=print
            print("gem5_configure: SLICC discovery warnings:\n" + result.stderr)
        repository_ctx.file(
            "configs/slicc_manifests.bzl",
            content = '"""SLICC manifests (empty -- discovery failed)."""\n\n' +
                      "SLICC_MANIFESTS = {}\n",
        )

def _gem5_configure_impl(repository_ctx):
    """Implementation of the gem5_configure repository rule.

    Creates the @gem5 repository by:
    1. Overlaying BUILD files from gem5-overlay/ onto @gem5-raw sources
    2. Extracting version info from source
    3. Running SLICC discovery to generate output manifests
    4. Writing static configuration to configs/vars.bzl
    """
    # Resolve paths. @gem5-raw points to the gem5 source root.
    src_root = repository_ctx.path(
        repository_ctx.attr.gem5_raw,
    ).dirname

    # The overlay directory is within the Bazel module (bazel/gem5-overlay/).
    overlay_root = repository_ctx.path(
        repository_ctx.attr.overlay_path,
    ).dirname

    # The module root is the parent of gem5-overlay/ (i.e., the bazel/ dir).
    module_root = overlay_root.dirname

    # Merge source tree with overlay BUILD files.
    _overlay_directories(repository_ctx, src_root, overlay_root)

    # Extract version info.
    version = _extract_version(repository_ctx, src_root)

    # Write static configuration (flag-independent only).
    repository_ctx.file(
        "configs/vars.bzl",
        content = '"""Generated by configure.bzl -- do not edit."""\n\n' +
                  'GEM5_VERSION = "{}"\n'.format(version),
    )

    # Generate SLICC output manifests for each protocol.
    _generate_slicc_manifests(repository_ctx, src_root, module_root)

gem5_configure = repository_rule(
    implementation = _gem5_configure_impl,
    local = True,
    configure = True,
    attrs = {
        "gem5_raw": attr.label(
            doc = "Label pointing to a file in @gem5-raw (used to resolve source root).",
        ),
        "overlay_path": attr.label(
            doc = "Label pointing to a file in the overlay directory (used to resolve overlay root).",
        ),
    },
)
