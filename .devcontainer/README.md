# Developing gem5 in a container

The configuration provides the tools to build and debug the checked-out
source. It does not install a separate gem5 executable. Build a workspace binary
with, for example, `scons build/ALL/gem5.opt -j2`.

The container uses the Ubuntu 24.04 environment used by most gem5 CI jobs.
Formatting is provided by the repository's pinned pre-commit hooks.

## Building the image

Bake connects the devcontainer directly to the Ubuntu 24.04 base target. Local
base-Dockerfile changes apply without first publishing a base image.

To build the image locally, run this from `util/dockerfiles`:

```sh
docker buildx bake devcontainer
```

A local single-platform build can be loaded into Docker for testing:

```sh
docker buildx bake devcontainer --load \
    --set '*.platform=linux/arm64'
```

Use `linux/amd64` on an x86 host. Both supported host architectures can simulate
guest architectures other than their own; RISC-V is not an image host platform.

## Python tooling and hooks

Setup installs `requirements.txt` into the workspace's ignored `.venv` and
initializes the pinned pre-commit hook environments, including clang-format.
The terminal and Python extension use this virtual environment. Distro Python
modules remain available for tools such as SCons and pydot; development-tool
pins are installed in the virtual environment rather than system Python.

After changing branches or requirements, rerun `.devcontainer/on-create.sh` to
refresh the tools and hooks. Setup is safe to repeat.

## Codespaces prebuilds and caches

Configure a Codespaces prebuild for `develop` in the repository's **Settings >
Codespaces > Prebuild configuration**. Use the default devcontainer and an
update frequency appropriate to usage. This is a repository setting and is not
enabled merely by merging these files.

`updateContentCommand` installs requirements and prepares hook environments on
prebuild updates. `postCreateCommand` installs the hooks in each user's checkout.
No simulator build is performed automatically; use the build tasks for the ISA
you need. If source builds are later added to prebuilds, put incremental work in
`updateContentCommand` and measure the compute and storage cost first.

Caches live in the ignored `.devcontainer-cache` directory in the workspace.
This preserves them across both Codespaces rebuilds and local container rebuilds
without relying on a writable parent directory or a particular repository name.
`GEM5_RESOURCE_DIR` selects its `resources` subdirectory; explicit resource paths
in a configuration or test still take precedence. Compiler calls use ccache with
a 2 GiB limit. Use `ccache --show-stats` to inspect it. Resource downloads and
pre-commit environments have no automatic size cap; remove unused entries when
space is needed. The workspace `.venv` is refreshed during content setup.

The existing 8-CPU, 16-GB-memory and 32-GB-storage requirements remain unchanged.
Large full-system resources or multiple build variants may need more storage;
select build parallelism for the available memory rather than blindly using
all CPUs.

## Editor tasks and debugging

Run **gem5: build** to choose a build configuration and job count. It builds
`gem5.opt` and generates a matching compilation database. **gem5: generate
compiler commands** generates just the database. Both tasks update an ignored
symlink used by C++ IntelliSense, so it follows the selected build configuration.

**gem5: list quick suites** lists the ALL-ISA quick TestLib suites. Copy a suite
UID into **gem5: run selected quick suite** to run it, including required builds.
This task does not skip builds or run every quick suite automatically.

Build the desired configuration, open a gem5 Python configuration script, then
choose **gem5: debug current configuration script** and the same build
configuration. GDB launches the workspace binary with the open script. Add
script-specific arguments to the launch configuration when needed. This debugs
simulator C++; it does not attach a Python debugger to embedded Python code.

## Container user and workspace ownership

The container runs development commands as `gem5`, with a writable home and
passwordless sudo for additional packages. VS Code can map this user's UID/GID
when opening a local Linux bind mount. Setup discovers the actual checkout
location and adds an exact Git trust entry only when ownership requires it;
forks and renamed directories do not need `/workspaces/gem5`.

## Publishing and refreshing the image

`devcontainer-build.yaml` builds the `devcontainer` Bake target separately from
the general Docker image workflow. It publishes a candidate using a unique tag
containing the UTC date, source revision, workflow run ID, and attempt. It then
creates the complete configuration on native amd64 and arm64 runners, including
Features and lifecycle commands, and runs `smoke-test.sh`. Only after both
architectures pass does it promote the candidate to `latest`.
The candidate tags remain available for identifying or selecting an exact build.
The registry cache is mutable; candidate tags are not reused by this workflow.

The smoke test checks non-root access, Python pins, hooks, writable caches,
compilation-database generation, a focused C++ unit test, and GDB.
It does not run the full simulator regression suite or validate VS Code's UI.

Dockerfile and development-configuration changes on `develop` trigger a build.
The workflow can also be dispatched manually. The scheduler adds a Monday
08:30 UTC refresh on `develop`. As with gem5's other scheduled workflows, the
scheduler and dispatched workflow must be present on `stable` (the default
branch) for scheduled operation; carry those workflow files through the normal
stable-branch maintenance process. This PR does not update `stable` itself.

To run the same smoke test in an existing development container:

```sh
.devcontainer/smoke-test.sh
```

Candidate builds bypass the Ubuntu dependency target's layer cache so APT checks
for updates even when the upstream Ubuntu image digest has not changed. The
devcontainer also receives a per-publication `PACKAGE_REFRESH` argument to
refresh its additional packages.

For local Docker Desktop use, a Linux volume for the checkout avoids bind-mount
ownership inconsistencies that can make Git reject newly created pre-commit
cache repositories. If setup reports dubious ownership for those repositories,
reopen the checkout in a container volume rather than disabling Git's ownership
checks globally. The native Linux Codespaces workspace does not use that macOS
bind-mount layer.
