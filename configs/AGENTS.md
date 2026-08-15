# Config Agent Notes

This directory contains simulation configuration scripts and examples. Many
are examples, compatibility helpers, or scripts for a specific test or study;
their presence does not make every helper a stable public API. Treat
`configs/example` as runnable examples, not as a single canonical interface to
copy blindly.

## Creating Simulations

When a task requires a new simulation configuration, prefer creating a focused
config file for that task and use the gem5 standard library in
`src/python/gem5` where practical. The stdlib provides higher-level boards,
processors, memory systems, cache hierarchies, resources, and simulator helpers
intended for user configuration scripts.

The stdlib is occasionally limited. It is acceptable to use lower-level
SimObject wiring when the experiment requires bare-bones control, unsupported
hardware composition, or behavior the stdlib cannot express. In that case,
keep the config narrow, document the reason for bypassing stdlib helpers, and
avoid turning examples into reusable interfaces.

## Legacy And Deprecated Configs

`configs/deprecated` contains old scripts retained for reference or migration.
Avoid extending deprecated scripts unless the task explicitly asks for legacy
support. If changing `configs/common` or `configs/ruby`, check existing users
and TestLib coverage because these helpers can affect many legacy examples.

## Validation

Run `python3 -m py_compile` for syntax-only checks where imports permit it.
Then run the narrowest configuration or TestLib suite that instantiates the
changed path; compilation alone does not prove that SimObject wiring, resource
requirements, or exit-event handling is correct.
