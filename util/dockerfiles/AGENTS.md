# Dockerfile Agent Notes

This directory defines gem5 CI and development images. Changes here often need
matching updates in `.github/workflows`, `util/dockerfiles/docker-bake.hcl`,
and documentation.

## Dependency Policy

gem5 supports current Ubuntu LTS releases. General dependency minimums should
track the oldest supported Ubuntu LTS defaults, while the newest supported LTS
must continue to work. Prefer distro packages when they satisfy policy.

For non-compiler dependencies, document floors as `major.minor+` where
practical, not full distro patch strings. Compiler support is tracked by GCC
and Clang major version.

## Compiler Images

When adding or raising supported GCC/Clang versions, keep compiler Dockerfiles,
`docker-bake.hcl`, `.github/workflows/compiler-tests.yaml`, CI container
references, and build documentation aligned. If a compiler package requires a
newer Ubuntu base, add the image and bake target as part of the same change.

## CI Reproduction

Docker images are central to gem5 testing because they provide stable,
reproducible environments. The Ubuntu all-dependency images are the common
baseline for CI dependency coverage. When a CI job fails, inspect the relevant
`.github/workflows` file and reproduce with the same container image where
practical.

GPU SE-mode tests currently require the `gcn-gpu` image. A GPU-capable
container does not by itself select GPU tests; TestLib still needs matching
`--host` and `--isa` filters.

## Validation

Use targeted checks before expensive builds:

```sh
docker buildx build --check -f util/dockerfiles/clang-compiler/Dockerfile .
docker buildx build --check -f util/dockerfiles/gcc-compiler/Dockerfile .
git diff --check
```

Avoid changing `gem5/ext/` to enforce dependency policy.
