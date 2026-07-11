# GitHub Workflow Agent Notes

This directory controls GitHub metadata and Actions workflows. Keep workflow
changes reviewable and mechanical.

## CI Surfaces

`ci-tests.yaml` is pull-request validation. `daily-tests.yaml`,
`weekly-tests.yaml`, and `compiler-tests.yaml` are broader scheduled or manual
coverage. Keep related changes aligned across workflows, Docker images, bake
targets, and documentation.

The `clang-format-check` job is advisory for merge validity: a failure does not
need to block a valid merge, but the formatting error should still be relayed
in the PR comment section whenever the check fails.

## Validation

Run `git diff --check` for all workflow edits. Parse YAML before finishing, for
example with Ruby or Python, and inspect the expanded matrix logic when a
change affects TestLib suite selection.

## Failure Investigation

Use the failing job log as evidence. Workflow-level failure status can hide the
real failing matrix entry. For GPU jobs, confirm both `--host` and `--isa`
selection; the container alone does not determine the selected gem5 build.

Docker containers define the intended CI environment. When reproducing a
failure, inspect the relevant workflow file and use the same image where
practical, especially for Ubuntu all-dependency jobs and GPU SE-mode jobs using
the `gcn-gpu` image.
