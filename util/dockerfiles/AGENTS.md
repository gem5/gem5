# Dockerfile Agent Notes

This directory defines gem5 CI and development images. Changes here often need
matching updates in `.github/workflows`, `util/dockerfiles/docker-bake.hcl`,
and documentation.

## Dependency Policy

gem5's documented policy is to support current Ubuntu LTS releases. General
dependency minimums should track the oldest supported Ubuntu LTS defaults,
while the newest supported LTS should continue to work. This policy is not
proof that every current LTS already has an image or CI job in this checkout;
verify `docker-bake.hcl`, workflows, Dockerfiles, and build documentation
before describing actual coverage. Prefer distro packages when they satisfy
policy.

For non-compiler dependencies, document floors as `major.minor+` where
practical, not full distro patch strings. Compiler support is tracked by GCC
and Clang major version.

## Compiler Images

When adding or raising supported GCC/Clang versions, keep compiler Dockerfiles,
`docker-bake.hcl`, `.github/workflows/compiler-tests.yaml`, CI container
references, and build documentation aligned. If a compiler package requires a
newer Ubuntu base, add the image and bake target as part of the same change.

## CI Reproduction

Docker images are central to gem5 testing because they standardize the build
environment. The Ubuntu all-dependency images are the common baseline for CI
dependency coverage. Mutable tags such as `latest` do not identify an exactly
reproducible environment; use the workflow revision and image digest when an
exact reproduction matters. When a CI job fails, inspect the relevant
`.github/workflows` file and reproduce with the same image where practical.

GPU SE-mode tests currently use the `gcn-gpu` image. A GPU-capable container
does not by itself select GPU tests; TestLib still needs the `gcn_gpu` host
filter. The current GPU suites declare `VEGA_X86`, so an ISA filter is optional
unless a narrower host-and-ISA intersection is desired.

## Validation

From `util/dockerfiles`, first inspect the resolved bake target and then lint a
specific Dockerfile with the arguments that target supplies:

```sh
docker buildx bake --print clang-version-19
docker buildx build --check --build-arg version=19 \
    -f clang-compiler/Dockerfile clang-compiler
docker buildx bake --print gcc-version-14
docker buildx build --check --build-arg version=14 \
    -f gcc-compiler/Dockerfile gcc-compiler
git diff --check
```

Replace the example versions with the targets being changed. `--check` parses
and lints the Dockerfile; it does not prove that packages install or that gem5
builds in the image. Build the affected target and run a focused gem5 build
when those behaviors are in scope.

Avoid changing `gem5/ext/` to enforce dependency policy.
