---
timeout-minutes: 5

on:
  workflow_dispatch:
    inputs:
      discussion_number:
        description: 'Discussion number'
        required: true
        type: string

permissions:
  discussions: read
  contents: read
  copilot-requests: write

tools:
  web-fetch:
  web-search:
  github:
    toolsets: [discussions, repos]
    lockdown: false

safe-outputs:
  add-comment:
    discussions: true
  noop:
    report-as-issue: false

engine:
  id: codex

model: gpt-5.6-terra

---

# Discussion responder

If discussion ${{ github.event.inputs.discussion_number }} is a question about using gem5, look through existing gem5 documentation at the links provided below and leave a comment on the discussion that answers the author's question.

The existing gem5 documentation can be found at the following links:

- The gem5 GitHub repo: https://github.com/gem5/gem5

- The GitHub repository for the gem5 website: https://github.com/gem5/website

Note: This workflow’s sandbox may not allow fetching content from the following links, as they are non-GitHub domains. If a link can’t be accessed, rely on the GitHub repos above and cite the exact file paths/URLs you used.

- The documentation on the gem5 website: https://www.gem5.org/documentation/

- The gem5 email archive: https://gem5.googlesource.com/public/gem5-website/+/7d6d1f73d4421941da646373c9e5ee4c3aba9a10/_pages/mailing_list.md

- The slides for the 2024 gem5 bootcamp: https://bootcamp.gem5.org/

- The gem5 v20 paper: https://arxiv.org/abs/2007.03152

- The original gem5 paper: https://dl.acm.org/doi/10.1145/2024716.2024718

If there are discrepancies between different sources of documentation, use the most recently updated piece of documentation.
