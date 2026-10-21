---
timeout-minutes: 5

on:
  workflow_dispatch:
    inputs:
      issue_number:
        description: 'Issue number'
        required: true
        type: string

concurrency:
  group: "test-failure-doctor-second-half-${{ github.event.inputs.issue_number }}"

permissions:
  issues: read
  contents: read
  copilot-requests: write

tools:
  bash: ["cat", "ls", "find", "grep", "head", "tail", "wc"]
  github:
    toolsets: [issues, labels, repos]
    min-integrity: none
  web-fetch:

safe-outputs:
  add-labels:
    allowed: [ext-testlib, fastmodel, gdb, github, good-first-contribution, gpu, gpu-compute, help wanted, learning-gem5, mem, mem-cache, mem-dram, mem-garnet, mem-ruby, misc, python, question, resources, resources-website, scons, sim, sim-se, stats, stdlib, system-arm, systemc, tests, util, util-docker, util-gem5art, util-m5, website]
    # invalid, low-priority, needs details, Stale, and wontfix excluded from this list
  add-comment: {}
  noop:
    report-as-issue: false

engine:
  id: copilot
  env:
    COPILOT_PROVIDER_BASE_URL: https://api.openai.com/v1
    COPILOT_MODEL: gpt-5-mini
    COPILOT_PROVIDER_API_KEY: ${{ secrets.COPILOT_PROVIDER_API_KEY }}


---

# Issue Labeler - Second Half

The following is the issue number of the issue that you should analyze:

Issue number: ${{ github.event.inputs.issue_number }}

Get the contents of the issue with this issue number in the current repository, then analyze the title and body of the opened issue.

Add zero or more of the allowed labels: `ext-testlib`, `fastmodel`, `gdb`, `github`, `good-first-contribution`, `gpu`, `gpu-compute`, `help wanted`, `learning-gem5`, `mem`, `mem-cache`, `mem-dram`, `mem-garnet`, `mem-ruby`, `misc`, `python`, `question`, `resources`, `resources-website`, `scons`, `sim`, `sim-se`, `stats`, `stdlib`, `system-arm`, `systemc`, `tests`, `util`, `util-docker`, `util-gem5art`, `util-m5`, `website`.

Consider the title of the issue to be more important than the body when deciding which labels to add. If the title starts with a comma separated list of labels, followed by a colon, then make sure that the labels listed in the title are applied if they are valid. If there isn't a comma separated list followed by a colon, but there are keywords corresponding to a label in the title, try to apply labels based on the keywords.
If something corresponding to a label only comes up 1-2 times in the body,
but isn't mentioned at all in the title, then don't add that label.

Look at the file MAINTAINERS.yaml in the top level of the gem5 repository for information on when some of these labels should be applied.

Each issue should only have a total of 1-3 labels, and should only have the most relevant labels applied. If a fourth label is truly necessary and relevant, an issue can have up to 4 labels.
This means that generally, this workflow should only apply 0-2 labels, or possibly 3 if none of the relevant labels are applied by the other workflow.
Keep in mind that this workflow only has some of the labels, and that another workflow will look through the rest of the labels and apply them if they are relevant.

The labels that the other workflow could add are as follows: `arch`, `arch-arm`, `arch-mips`, `arch-power`, `arch-riscv`, `arch-sparc`, `arch-vega`, `arch-x86`, `base`, `base-stats`, `bug`, `build error`, `classic caches`, `configs`, `cpu`, `cpu-kvm`, `cpu-minor`, `cpu-o3`, `cpu-simple`, `dependencies`, `dev`, `dev-amdgpu`, `dev-arm`, `dev-hsa`, `dev-virtio`, `doc`, `duplicate`, `enhancement`, `ext`. If some of these labels are more relevant than the labels that this workflow can apply, then apply fewer labels in this workflow and allow the other workflow to apply the more relevant labels. Prioritize keeping the number of labels to 3 or less, and do not apply labels for every detail in the issue.

The list of labels that can be applied to PRs and their corresponding file
globs relative to the `gem5` directory can be found at the following link:
https://github.com/gem5/gem5/blob/stable/.github/labeler.yml. If the issue
explicitly mentions a file path that matches one of the globs in `labeler.yml`,
then this adds additional certainty that the label should be applied.

Some labels should only be applied if the issue seems to be about a problem with
the implementation or use of the label's subject in gem5; these labels should
not be applied if the issue only mentions them in relation to describing or
reproducing the problem. The following list contains labels that this
description applies to, and which can be applied by this workflow:

- `gdb`: Only apply this label if the issue seems related to using gdb with
gem5. Do not include the `gdb` label if the issue is only using gdb to
demonstrate an issue with something else.

- The `help wanted` label isn't related to gem5, but its use should also be
limited. Only apply this label if the author of the issue asked a question about
how to use gem5, gem5's implementation, or something else that indicates that
the author was specifically looking for help from others. Do not add this label
to a normal bug report that only reports a problem.

- `learning-gem5`: This label should only be applied to issues that seem to be
about problems with the gem5 learning resources provided under
`src/learning_gem5/` in the gem5 repository.

- `python`: Don't apply this label unless the issue appears to be about issues
with the Python side of gem5 or interactions between Python and gem5. Do not
apply the `python` label solely because Python output or scripts were included
in the issue.

- The `resources`, `resources-website`, `website` labels should only be applied
to issues describing problems with gem5 resources, the gem5 resources website,
and the gem5 website, respectively.

- `sim`: Don't apply unless the filepath `src/sim` is mentioned in the issue,
`sim` is included in the title of the issue, or the **main topic** of the issue
is about problems with simulation in general in gem5. Do not add this label if
the issue is a bug report and does not include `sim` or `src/sim` in the title
or body. Most issues related to gem5 will involve issues with simulation, so do
not apply the `sim` label unless these specific conditions are met.

- `stats`: This label should only applied to issues about how gem5 measures
statistics; do not apply this label if, for instance, the issue supplies
stats to show why the issue is a problem.

- `system-arm`: Only apply if the issue seems to be related to files or code
located in the `system/arm` subdirectory of the `gem5` repo.

- `tests`: This label should only be applied to issues about gem5's tests or
testing system. Do not apply this label if the issue uses gem5's testing system
to show how to reproduce an error. Furthermore, do not apply this issue solely
because the issue provides a testing script, unless the issue is specifically
about adding tests to gem5.

The labels `arch`, `base`, `dev`, `cpu`, `ext`, `mem`, `python`, `sim`, and `util`, are all "general" labels that have "more specific" labels. "More specific" labels are labels that start with one of the general labels, followed by a dash (-) and another word. For example, `arch-arm` would be one of the more specific labels for `arch`. Apply the more specific label if possible (i.e. if it is relevant and is one of the labels this workflow is allowed to add), and don't apply the general label if a more specific label is applied. For example, if the label `arch-arm` can be applied, then apply that label, and don't apply `arch`. If keywords corresponding to multiple "more specific" labels appear in an issue, apply each of the "more specific" labels, and don't apply the "general" label. For example, if the labels `cpu-simple` and `cpu-o3` can both be applied, apply `cpu-simple` and `cpu-o3`, and don't apply `cpu`.

- The more specific label for `python` is `stdlib`, which should be applied to files in `src/python/gem5`.
