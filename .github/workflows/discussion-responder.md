---
timeout-minutes: 5

on:
  workflow_dispatch:
    inputs:
      discussion_number:
        description: 'Discussion number'
        required: true
        type: string
      discussion_category:
        description: 'Discussion category'
        required: true
        type: string

concurrency:
  group: "discussion-responder"
  job-discriminator: ${{ github.event.inputs.discussion_number }}

permissions:
  discussions: read
  contents: read
  copilot-requests: write

tools:
  bash: ["*"]
  web-fetch:
  web-search:
  github:
    toolsets: [discussions, repos]
    min-integrity: none

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

If discussion ${{ github.event.inputs.discussion_number }} contains question(s)
about using gem5 or how something is implemented in gem5, look through existing
gem5 documentation at the links provided below and leave a comment on the
discussion that answers the author's question(s).

Check if the discussion category, ${{ github.event.inputs.discussion_category }},
is `gem5-dev`.

If the discussion category is `gem5-dev`, the discussion might not contain a
question about using gem5 or how a pre-existing feature is implemented in gem5.
In that case, call `noop` and exit without leaving a comment. Furthermore,
discussions with the category `gem5-dev` may contain questions that ask the
community or gem5 developers for their opinion, such as asking if adding a
certain feature would be useful or not. Do not leave comments answering these
questions. Only answer technical questions about gem5, such as how to use gem5,
or how an existing feature is implemented in gem5.

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
