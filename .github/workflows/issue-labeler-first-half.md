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
  group: "test-failure-doctor-first-half-${{ github.event.inputs.issue_number }}"

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
    allowed: [arch, arch-arm, arch-mips, arch-power, arch-riscv, arch-sparc, arch-vega, arch-x86, base, base-stats, bug, build error, classic caches, configs, cpu, cpu-kvm, cpu-minor, cpu-o3, cpu-simple, dependencies, dev, dev-amdgpu, dev-arm, dev-hsa, dev-virtio, doc, duplicate, enhancement, ext]
    # agentic-workflows excluded from this list
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

# Issue Labeler - First Half

The following is the issue number of the issue that you should analyze:

Issue number: ${{ github.event.inputs.issue_number }}

Get the contents of the issue with this issue number in the current repository, then analyze the title and body of the opened issue.

Add zero or more of the allowed labels: `arch`, `arch-arm`, `arch-mips`, `arch-power`, `arch-riscv`, `arch-sparc`, `arch-vega`, `arch-x86`, `base`, `base-stats`, `bug`, `build error`, `classic caches`, `configs`, `cpu`, `cpu-kvm`, `cpu-minor`, `cpu-o3`, `cpu-simple`, `dependencies`, `dev`, `dev-amdgpu`, `dev-arm`, `dev-hsa`, `dev-virtio`, `doc`, `duplicate`, `enhancement`, `ext`.

Consider the title of the issue to be more important than the body when deciding which labels to add. If the title starts with a comma separated list of labels, followed by a colon, then make sure that the labels listed in the title are applied if they are valid. If there isn't a comma separated list followed by a colon, but there are keywords corresponding to a label in the title, try to apply labels based on the keywords.
If something corresponding to a label only comes up 1-2 times in the body,
but isn't mentioned at all in the title, then don't add that label.

Look at the file MAINTAINERS.yaml in the top level of the gem5 repository for information on when some of these labels should be applied.

Each issue should only have a total of 2-3 labels, and should only have the most relevant labels applied. If a fourth label is truly necessary and relevant, an issue can have up to 4 labels.
This means that generally, this workflow should only apply 1-2 labels, or possibly 3 if none of the relevant labels are applied by the other workflow.
Keep in mind that this workflow only has some of the labels, and that another workflow will look through the rest of the labels and apply them if they are relevant.

The labels that the other workflow could add are as follows: `ext-testlib`, `fastmodel`, `gdb`, `github`, `good-first-contribution`, `gpu`, `gpu-compute`, `help wanted`, `learning-gem5`, `mem`, `mem-cache`, `mem-dram`, `mem-garnet`, `mem-ruby`, `misc`, `python`, `question`, `resources`, `resources-website`, `scons`, `sim`, `sim-se`, `stats`, `stdlib`, `system-arm`, `systemc`, `tests`, `util`, `util-docker`, `util-gem5art`, `util-m5`, `website`. If some of these labels are more relevant than the labels that this workflow can apply, then apply fewer labels in this workflow and allow the other workflow to apply the more relevant labels. Prioritize keeping the number of labels to 3 or less, and do not apply labels for every detail in the issue.

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

- `configs`: Only apply the `configs` label if the issue seems related to
configuration scripts provided in the gem5 repository. Do not apply this label
solely because the issue includes a configuration script (for instance, to help
reproduce the issue).

- `cpu`: Only apply `cpu` to issues that are about the general cpu
implementation in gem5.

- `dependencies`: Only apply the `dependencies` label if the issue is
about issues with gem5's dependencies. Don't apply the `dependencies` label if,
for instance, the issue mentions needing certain dependencies to reproduce the
issue.

- `dev`: Only apply `dev` to issues that seem related to files found under the
`src/dev/` directory.

The labels `arch`, `base`, `dev`, `cpu`, `ext`, `mem`, `python`, `sim`, and `util`, are all "general" labels that have "more specific" labels. "More specific" labels are labels that start with one of the general labels, followed by a dash (-) and another word. For example, `arch-arm` would be one of the more specific labels for `arch`. Apply the more specific label if possible (i.e. if it is relevant and is one of the labels this workflow is allowed to add), and don't apply the general label if a more specific label is applied. For example, if the label `arch-arm` can be applied, then apply that label, and don't apply `arch`. If keywords corresponding to multiple "more specific" labels appear in an issue, apply each of the "more specific" labels, and don't apply the "general" label. For example, if the labels `cpu-simple` and `cpu-o3` can both be applied, apply `cpu-simple` and `cpu-o3`, and don't apply `cpu`.

- The more specific label for `python` is `stdlib`, which should be applied to files in `src/python/gem5`.
