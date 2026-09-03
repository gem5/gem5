# Copilot Instructions

Use the repository guidance in `../AGENTS.md` as the source of truth. Important
defaults for generated changes:

- Target contributor work at `develop`; `stable` is release-only except for
  maintainer-directed hotfixes.
- Use GitHub issues or GitHub Discussions where issue context is needed.
- Keep gem5 commit subjects tagged with `MAINTAINERS.yaml` components and wrap
  body text to 72 columns.
- Do not edit generated files under `build/`.
- Avoid changing `gem5/ext/` for dependency or policy work unless explicitly
  directed.
- For tests, distinguish C++ GTests, Python PyUnit tests, and TestLib
  quick/long/very-long suites.
- For CI/debugging, inspect concrete job logs and SCons config logs before
  assigning root cause.
- `clang-format-check` does not need to pass for a valid merge, but if it fails
  the error should be relayed to the PR comment section.
