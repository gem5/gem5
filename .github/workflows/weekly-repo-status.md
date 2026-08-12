---
description: |
  This workflow creates weekly repo status reports. It gathers recent repository
  activity (issues, PRs, discussions, code changes) and generates a summary GitHub issue.

on:
  workflow_dispatch:

permissions:
  contents: read
  issues: read
  pull-requests: read
  discussions: read
  copilot-requests: write

network: defaults

tools:
  github:
    # If in a public repo, setting `lockdown: false` allows
    # reading issues, pull requests and comments from 3rd-parties
    # If in a private repo this has no particular effect.
    lockdown: false
    min-integrity: none # This workflow is allowed to examine and comment on any issues

safe-outputs:
  mentions: false
  allowed-github-references: []
  create-issue:
    title-prefix: "misc: "
    labels: [misc]
    close-older-issues: true
  report-failure-as-issue: false

engine:
  id: copilot
  env:
    COPILOT_PROVIDER_BASE_URL: https://api.openai.com/v1
    COPILOT_MODEL: gpt-5.6-terra
    COPILOT_PROVIDER_API_KEY: ${{ secrets.COPILOT_PROVIDER_API_KEY }}

---

# Weekly Repo Status

Create a weekly status report for the gem5/gem5 repo as a GitHub issue.
When "weekly" or "in the last week" is used, and the workflow was automatically launched (i.e. triggered by scheduler.yaml), it specifically refers to the time since the last automated run of this workflow, if the last automated run was between 1-2 weeks ago. If the last automated run was 2+ weeks ago, or was less than 1 week ago, use the same criteria as below, as if the workflow was launched by a GitHub user.

If this workflow was launched by a GitHub user, you should include all activity starting from **midnight, or the beginning, of the day** that is exactly a week before the workflow was launched, in the Pacific Time Zone.

For example, if this workflow is run on May 19th at 9am PT, then the summary should include all activity between May 12th at 12am PT and May 19th at 9am PT.

- In the section below, the words "item" or "items" refers to PRs, issues, and discussions.

- In the section below, you will be asked to make tables which will include PR/issue/discussion titles. **Do not** truncate or summarize the titles; print them exactly as they are.

## What to include

- An executive summary of what has been done in the last week.

- Tabless of PRs, issues, and discussions that were opened in the last week, and a short summary of each. This section should use the heading `## Opened in the last week`. Only include items that are still open. PRs, issues, and discussions should be sorted into separate tables, and within each table, items should be sorted from newest to oldest, i.e. by descending PR/issue/discussion number. These tables should be formatted as tables with the PR/issue/discussion number, title, author, and summary. Key words and phrases in the summary should be bolded. Use the table template shown below:

| # | Title | Author | Summary |
|---|-------|--------|---------|

- Tables of PRs, issues, and discussions that were modified, had new comments added, or had other activity in the last week. This section should use the heading `## Active in the last week`. Provide a short summary of what the changes were, and what the activity was. Be specific about **who** made the changes/added comments/etc, and **what** the activity was. Key words and phrases in the summary should be bolded. The summary should not include activity from previous runs of this workflow that mention the PR/issue/discussion. The summaries also should not mention runs of CI tests for PRs, unless there were test failures.

The PRs, issues, and discussions in this section should be as though you filtered them by the `updated` qualifier between the current time and a week ago, and excluded the ones that were `created` between the current time and a week ago.

There should be two tables in total. The first table should contain items that were created in the last 2-6 weeks, and should be preceded by the heading `### Opened in the last 2-6 weeks`. The second table should contain items created over 6 weeks ago, and should be preceded by the heading `### Opened 6+ weeks ago`. Pay special attention to older, formerly inactive items that had recent activity.

Within each table, items should first be **grouped by whether they are PRs, issues, or discussions**, then sorted in order of descending PR/issue/discussion number **within each group**. For instance, all PRs should be listed first, then all issues, then all discussions. All PRs should be sorted in descending order of PR number, issues by descending order of issue number, and all discussions by descending order of discussion number. In this ordering, the first issue number can be larger than the last PR number, though it might not be.

The tables should use the format below, and have the PR/issue/discussion number (in the `| # |` column), whether the item is a PR/issue/discussion (in the `| Type |` column), the title, author, person/people who updated, and summary. The PR/issue/discussion number should link to the PR/issue/discussion. The `Author` and `Updated by` fields should link to the user's GitHub profile.

| # | Type | Title | Author | Updated by | Summary |
|---|------|-------|--------|------------|---------|

- A table of PRs that were last updated in the last six weeks, but haven't had any activity in the last two weeks. This section should be preceded by the heading `## PRs with no activity in the last 2 weeks`, and this heading should be followed by the caption `These PRs were created or updated in the last 6 weeks, but haven't had any activity in the last 2 weeks.`. This table should have the PR number, title, author, a summary of the changes made, and the status of the PR, e.g. if it's been waiting for a response from the author or reviewer for two weeks or more, if two weeks or more have passed with no activity since the PR was opened, etc. The summary for a PR should not include activity from previous runs of this workflow that mention the PR. Sort PRs by descending order of PR number.
This table should be formatted as follows:

| PR # | Title | Author | Summary | Status |
|------|-------|--------|---------|--------|

In the `Status` column, include *what the last activity was*, *who the last activity was from* and *how long it has been since the last activity*. The GitHub usernames in the `Author` column should link to the authors' GitHub profiles.

- A table of issues, PRs, and discussions that might be high priority. For this table, it is fine to include items that were already listed in previous sections.
  - An issue might be high priority if:
    - several community members have commented on it and said that they have encountered the same issue, especially if the issue causes the simulation to crash or produce inaccurate results.
    - One of the gem5 developers was pinged on the issue. The GitHub usernames of the gem5 developers are `BobbyRBruce`, `Harshil2107`, `erin-le`, and `powerjg`.

  - A PR might be high priority if:
    - One of the gem5 developers has been pushing commits to it. The GitHub usernames of the gem5 developers are as follows: `erin-le`, `Harshil2107`, `BobbyRBruce`, `powerjg`.
    - If the PR has been marked for inclusion in the next release
    - If the PR is a fix for a high priority issue
    - One of the gem5 developers was pinged on the PR, especially if one of the gem5 developers had previously commented on or reviewed the PR. The GitHub usernames of the gem5 developers are as follows: `erin-le`, `Harshil2107`, `BobbyRBruce`, `powerjg`.

  - A discussion might be high priority if:
    - several community members have commented on the discussion
    - One of the gem5 developers was pinged on the discussion. The GitHub usernames of the gem5 developers are `BobbyRBruce`, `Harshil2107`, `erin-le`, and `powerjg`.

  - Organize this table so items are grouped by whether they're PRs, issues, or discussions. Within each group, sort items in descending order of PR/issue/discussion number. For instance, all PRs should be listed first, then all issues, then all discussions. All PRs should be sorted in descending order of PR number, issues by descending order of issue number, and all discussions by descending order of discussion number. In this ordering, the first issue number can be larger than the last PR number, though it might not be. The GitHub usernames in the `Author` column should link to the authors' GitHub profiles. Use the table template shown below:

| # | Type | Title | Author | Why High Priority | Actions Needed |
|---|------|-------|--------|-------------------|----------------|

## Style

- The title of the summary issue should use the following format:
`Weekly Repo Status: {month} {start_day} - {month} {end_day}, {year}`, where the words enclosed in curly brackets should be swapped out for the appropriate days, months, and year. For example, if this workflow is run on May 19th, 2026, then it will contain activity starting from May 12th, so the title should be `Weekly Repo Status: May 12 - May 19, 2026`.
- Be concise - adjust length based on actual activity
- Be positive, encouraging, and helpful
- Bold key words in summaries of issues/PRs

## Process

1. Gather recent activity from the repository
2. Study the repository and its issues, pull requests, and discussions.
3. Create a new GitHub issue with your findings and insights
