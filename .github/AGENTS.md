# GitHub Workflow Agent Notes

This directory controls GitHub metadata and Actions workflows. Keep workflow
changes reviewable and mechanical.

## CI Surfaces

`ci-tests.yaml` runs for non-draft pull-request updates. `daily-tests.yaml`,
`weekly-tests.yaml`, and `compiler-tests.yaml` expose `workflow_dispatch`; the
cron-driven `scheduler.yaml` dispatches them on their daily or weekly cadence.
Keep related changes aligned across workflows, Docker images, bake targets,
and documentation.

The aggregate `ci-tests` job does not depend on `clang-format-check`, so the
workflow dependency graph does not make formatting a prerequisite for that
aggregate job. Branch-protection settings live on GitHub and can change; check
them before claiming any individual job is required or advisory. Regardless, a
formatting failure is evidence that should be reported to the contributor.

## Validation

Run `git diff --check` for all workflow edits and validate syntax with
`pre-commit run check-yaml --files <workflow-files>`. A YAML parser cannot
validate GitHub expressions or job semantics, so also inspect `needs`, `if`,
matrix expansion, permissions, and the exact shell commands changed.

## Failure Investigation

Use the failing job log as evidence. Workflow-level failure status can hide the
real failing matrix entry. For GPU jobs, confirm the effective host and ISA
selection; the container alone does not determine the selected gem5 build.

Docker containers define the intended CI environment. When reproducing a
failure, inspect the relevant workflow revision and use the same image digest
where available; a mutable `:latest` tag may no longer match an older run. This
especially matters for Ubuntu all-dependency jobs. GPU full-system suites use
that general dependency environment and run with the `ALL` TestLib build.
