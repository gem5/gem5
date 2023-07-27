This document serves as a guide to contributing to gem5.
The following subsections outline, in order, the steps involved in contributing
to the gem5 project.

## Determining what you can contribute

The easiest way to see how you can contribute to gem5 is to check our Jira
issue tracker: <https://gem5.atlassian.net> or GitHub issue tracker:
<https://github.com/gem5/gem5/issues>.

Browse these open issues and see if there are any which you are capable of
handling. When you find a task you are happy to carry out, verify no one else
is presently assigned, then leave a comment asking if you may assign yourself
this task. Though not mandatory, we
advise first-time contributors do this so developers more familiar with the
task may give advice on how best to implement the necessary changes.

Once a developers has replied to your comment (and given any advice they may
have), you may officially assign yourself the task. This helps the gem5
development community understand which parts of the project are presently being
worked on.

**If, for whatever reason, you stop working on a task, please unassign
yourself from the task.**

## Obtaining the git repo

The gem5 git repository is hosted at <https://github.com/gem5/gem5>.
**Please note: contributions made to other gem5 repos
will not be considered. Please contribute to <https://github.com/gem5/gem5>
exclusively.**

To pull the gem5 git repo:

```sh
git clone https://github.com/gem5/gem5
```

If you wish to use gem5 and never contribute, this is fine. However, to
contribute, we use the [GitHub Pull-Request model](https://docs.github.com/en/pull-requests), and therefore recommend [Forking the gem5 repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) prior to contributing.

### Forking

Please consult the [GitHub documentation on Forking a GitHub repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo).
As we will be working atop the `develop` branch, please ensure you Fork all the repository's branches, not just the `stable` branch.

This will create your own forked version of the gem5 repo on your own GitHub account.
You may then obtain it locally using:

```sh
git clone https://github.com/{your github account}/gem5
```

### stable / develop branch

When cloned the git repo will have the `stable` branch checked-out by default. The
`stable` branch is the gem5 stable release branch. I.e., the HEAD
of this branch contains the latest stable release of gem5. (execute `git tag`
on the `stable` branch to see the list of stable releases. A particular
release may be checked out by executing `git checkout <release>`). As the
`stable` branch only contains officially released gem5 code **contributors
should not develop changes on top of the `stable` branch** they should instead
**develop changes on top of the `develop` branch**.

To switch to the `develop` branch:

```sh
git switch develop
```

The develop `branch` is merged into the `stable` branch upon a gem5 release.
Therefore, any changes you make exist on the develop branch until the next release.

We strongly recommend creating your own local branches to do changes.
The flow of development works best if `develop` and `stable` are not modified directly.
This helps keep your changes organized across different branches in your forked repository.
The following example will create a new branch, from `develop`, called `new-feature`:

```sh
git switch -c new-feature
```

## Making modifications

### C/CPP

Different tasks will require the project to be modified in different ways.
Though, in all cases, our style-guide must be adhered to. The full C/C++ style
guide is outlined [here](/documentation/general_docs/development/coding_style).

As a high-level overview:

* Lines must not exceed 79 characters in length.
* There should be no trailing white-space on any line.
* Indentations must be 4 spaces (no tab characters).
* Class names must use upper camel case (e.g., `ThisIsAClass`).
* Class member variables must use lower camel case (e.g.,
`thisIsAMemberVariable`).
* Class member variables with their own public accessor must start with an
underscore (e.g., `_variableWithAccessor`).
* Local variables must use snake case (e.g., `this_is_a_local_variable`).
* Functions must use lower camel case (e.g., `thisIsAFunction`)
* Function parameters must use snake case.
* Macros must be in all caps with underscores (e.g., `THIS_IS_A_MACRO`).
* Function declaration return types must be on their own line.
* Function brackets must be on their own line.
* `for`/`if`/`while` branching operations must be followed by a white-space
before the conditional statement (e.g., `for (...)`).
* `for`/`if`/`while` branching operations' opening bracket must be on the
same line, with the closing bracket on its own line (e.g.,
`for (...) {\n ... \n}\n`). There should be a space between the condition(s)
and the opening bracket.
* C++ access modifies must be indented by two spaces, with method/variables
defined within indented by four spaces.

Below is a simple toy example of how a class should be formatted:

```C++
#DEFINE EXAMPLE_MACRO 7
class ExampleClass
{
  private:
    int _fooBar;
    int barFoo;

  public:
    int
    getFooBar()
    {
        return _fooBar;
    }

    int
    aFunction(int parameter_one, int parameter_two)
    {
        int local_variable = 0;
        if (true) {
            int local_variable = parameter_one + parameter_two + barFoo
                               + EXAMPLE_MACRO;
        }
        return local_variable;
    }

}
```

### Python

We use [Python Black](https://github.com/psf/black) to format our Python code
to the correct style. To install:

```sh
pip install black
```

Then run on modified/added python files using:

```sh
black <files/directories>
```

For variable/method/etc. naming conventions, please follow the [PEP 8 naming
convention recommendations](
https://peps.python.org/pep-0008/#naming-conventions). While we try our best to
enforce naming conventions across the gem5 project, we are aware there are
instances where they are not. In such cases please **follow the convention
of the code you are modifying**.

### Using pre-commit

To help enforce our style guide we use use [pre-commit](
https://pre-commit.com). pre-commit is a git hook and, as such, must be
explicitly installed by a gem5 developer.

To install the gem5 pre-commit checks, execute the following in the gem5
directory:

```sh
pip install pre-commit
pre-commit install
```

Once installed pre-commit will run checks on modified code prior to running the
`git commit` command (see [our section on committing](#committing) for more
details on committing your changes). If these tests fail you will not be able to
commit.

These same pre-commit checks are run as part our CI checks (those
which must pass in order for a change to be merged into the develop branch). It
is therefore strongly recommended that developers install pre-commit to catch
style errors early.

## Compiling and running tests

The minimum criteria for a change to be submitted is that the code is
compilable and the test cases pass.

The following command both compiles the project and runs our "quick"
system-level checks:

```sh
cd tests
./main.py run
```

**Note: These tests can take several hours to build and execute. `main.py` may
be run on multiple threads with the `-j` flag. E.g.: `python main.py run
-j6`.**

The unit tests should also pass. To run the unit tests:

```sh
scons build/NULL/unittests.opt
```

To compile an individual gem5 binary:

```sh
scons build/ALL/gem5.opt
```

This compiles a gem5 binary containing "ALL" ISA targets. For more information
on building gem5 please consult our [building documentation](
/documentation/general_docs/building).

## Committing

When you feel your change is done, you may commit. Start by adding the changed
files:

```Shell
git add <changed files>
```

Make sure these changes are being added to your forked repository.
Then commit using:

```Shell
git commit
```

The commit message must adhere to our style. The first line of the commit is
the "header". The header starts with a tag (or tags, separated by a comma),
then a colon. Which tags are used depend on which components of gem5
you have modified. **Please refer to the [MAINTAINERS.yaml](
https://github.com/gem5/gem5/blob/stable/MAINTAINERS.yaml) for
a comprehensive list of accepted tags**. After this colon a short description
of the commit must be provided. **This header line must not exceed 65
characters**.

After this, a more detailed description of the commit can be added. This is
inserted below the header, separated by an empty line. Including a description
is optional but it's strongly recommended. The description may span multiple
lines, and multiple paragraphs. **No line in the description may exceed 72
characters.**

To improve the navigability of the gem5 project we would appreciate if commit
messages include a link to the relevant Jira issue/issues.

Below is an example of how a gem5 commit message should be formatted:

```
test,base: This commit tests some classes in the base component

This is a more detailed description of the commit. This can be as long
as is necessary to adequately describe the change.

A description may spawn multiple paragraphs if desired.

Jira Issue: https://gem5.atlassian.net/browse/GEM5-186
```

If you feel the need to change your commit, add the necessary files then
_amend_ the changes to the commit using:

```sh
git commit --amend
```

This will give you opportunity to edit the commit message.

You may continue to add more commits as a chain of commits to be included in the pull-request.
However, we recommend that pull-requests are kept small and focused.
For example, if you wish to add a different feature or fix a different bug, we recommend doing so in another pull requests.

## Keeping your forked and local repositories up-to-date

While working on your contribution, we recommend keeping your forked repository in-sync with the source gem5 repository.
To do so, regularly [Sync your fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/syncing-a-fork).
This can be done via the GitHub web interface and, if so, you should `git pull` on top of your local `stable` and `develop` branches to ensure your local repository is in-sync.
To do so from the command line:

```sh
# Add the main gem5 repository as a remote on your local repository. This only
# needs done once.
git remote add upstream https://github.com/gem5/gem5.git

git fetch upstream # Obtain the latest from the gem5 repo.
git switch develop # Switch to the develop branch.
git merge upstream/develop # Merge the latest changes into the develop branch.
git push # Push to develop to your forked repo.
git switch stable # Switch to the stable branch.
git merge upstream/stable # Merge the latest changes into the stable branch.
git push # Push the changes to stable to your forked repo.
```

As our local branch work atop the `develop` branch, once we've synced our forked repository, we can rebase our local branch on top of the `develop` branch.
Assuming our local branch is called `new-feature`:

```sh
git switch develop # Switching back to the develop branch.
git pull # Ensuring we have the latest from the forked repository.
git switch new-feature # Switching back to our local branch.
git rebase develop # Rebasing our local branch on top of the develop branch.
```

Conflicts may need resolved between your branch and new changes.

## Pushing and creating a pull request

Once you have completed your changes locally, you can push to your forked gem5 repository.
Assuming the branch we are working on is `new-feature`:

```sh
git switch new-feature # Ensure we are on the 'new-feature' branch.
git push --set-upstream origin new-feature
```

Now, via the GitHub web interface, you can [create a pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) of your changes from your forked repository's branch into the gem5 `develop` branch.

## Passing the checks

Once you have created a pull request, the gem5 Continuous Integration (CI) tests will run.
These run a series of checks to ensure your changes are valid.
These must pass before your changes can be merged into the gem5 `develop` branch.

In addition to the CI tests, your changes will be reviewed by the gem5 community.
Your pull-request must have the approval of at least one community member prior to being merged.

Once your pull-request has passed all the CI tests and has been approved by at least one community member, it will be merged a gem5 maintainer will do a [Merge](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/about-pull-request-merges) on the pull-request.
The gem5 maintainers are individuals granted the ability to merge pull requests into the gem5 `develop` branch.


### Making iterative improvements based on feedback

A reviewer will ask questions and post suggestions on GitHub. You should read
these comments and answer these questions. **All communications between
reviewers and contributors should be done in a polite manner. Rude and/or
dismissive remarks will not be tolerated.**

When you understand what changes are required make amendments to the pull
request by adding patches to the same branch and then pushing to the forked repository.
A git "force push" (i.e., `git push --force`) is also acceptable if you wish to alter the commits locally in order to make the changes.
We encourage contributors to help keep our `git log` clean and readable.
We recommend that users rebase their changes frequently on top of the develop branch, squash their commits where appropriate (e.g., in cases where there are many small fix commits to a change in the same PR) then force push changes to keep their PR commits concise.

Once pushed to the forked repository, the pull request will automatically update with your changes.
The reviewer will then re-review your changes and, if necessary, ask for further changes, or approve your pull-request.

## Reviewing other contributions

We encourage all gem5 developers to review other's contributions.
Anyone may review a gem5 change and, if they feel it is ready, approve it.
All pull-requests can be found at <https://github.com/gem5/gem5/pulls>.

When reviewing a pull request we enforce the followings guidelines.
These have been designed to ensure clear and polite communication between all parties:

* In all forms of communication, contributors and reviewers must be polite.
Comments seen as being rude or dismissive will not be tolerated.
* If choosing to not approve a PR, please state clearly why.
When asking for changes, the commits should be specific and actionable.
General criticisms which cannot be addressed or understood by the contributor are unhelpful.
If the contribution needs improvement, reviewers should state what their requested changes are.
If more information is needed for the reviewers to make a decision the reviewer should ask clear questions.
If the PR is generally not seen as a worthwhile contribution, a good justification should be given so the contributor may fairly rebuttal.
* By default, the original contributor is assumed to own a change.
I.e., they are assumed to be the sole party to submit patches to the pull request.
If someone other than the original contributor wishes to submit patches on the original contributors behalf they should first ask permission.
Pull requests which appear abandoned may be adopted by a new contributor as long as there is good enough reason to assume the original contributor is no longer working on the pull request.
* Maintainers have the final say on whether a change is merged.
Your review will be taken into account by the maintainer.
It is expected, in all but the most extreme cases, that the reviewer's concerns must be addressed and for the reviewer to approve the the contribution prior to the maintainer merging the pull request.

We also recommend consulting Google's ["How to write code review comments"](https://google.github.io/eng-practices/review/reviewer/comments.html) for advice on giving feedback to contributors.

## Releases

gem5 releases occur 3 times per year. The procedure for releasing gem5 is as
follows:

1. Developers will be notified, via the gem5-dev mailing list, that a new
release of gem5 will occur. This should be no sooner than 2 weeks prior to the
creation of the staging branch (the first step in releasing a new version of
gem5). This gives time for developers to ensure their changes for the next
release are submitted to the develop branch.
2. When a release is ready, a new staging branch shall be created by a project
maintainer, from develop, with the name "release-staging-{VERSION}". The
gem5-dev mailing list will be notified that the staging branch will be merged
into the stable branch after two weeks, thus marking the new release.
3. The staging branch will have the full suite of gem5 tests run on it to
ensure all tests pass and the to-be-released code is in a decent state.
4. If a user submits a pull request to the staging branch, it will be considered
and undergo the standard github review process. However, only alterations that
cannot wait until the following release will be accepted for submission into
the branch (i.e., submissions to the staging branch for "last minute"
inclusions to the release should be of a high priority, such as a critical bug
fix). The project maintainers will use their discretion in deciding whether a
change may be submitted directly to the staging branch. All other submissions
to gem5 will continue to be made to the develop branch. Patches submitted
into the staging branch do not need to be re-added to the develop branch.
5. Once the staging branch has been deemed ready for release, the [release procedures](https://www.gem5.org/documentation/general_docs/development/release_procedures/) will be carried out.
This will end with the staging branch being merged into the stable branch.
6. The stable branch shall be tagged with the correct version number for that
release. gem5 conforms to a "v{YY}.{MAJOR}.{MINOR}.{HOTFIX}" versioning system.
E.g., the first major release of 2022 will be "v22.0.0.0", followed by
"v22.1.0.0". All the releases (with the exception of hotfixes) are considered
major releases. For the meantime, there are no minor releases though we keep
the minor release numbers in case this policy changes in the future.
7. The gem5-dev and gem5-user mailing lists shall be notified of the new gem5
release.

### Exemptions

Due to limitations with GitHub we may update the ".github" directory in the gem5 repo's `stable` branch between gem5 releases.
This is due to certain processes carried out by the GitHub Actions infrastructure which rely on configurations being present on a repository's primary branch.
As the files in ".github" only influence the functionality of our GitHub actions and other GitHub activities, updating these files does not change the functionality of the gem5 in way.
It is therefore safe to do this.
Despite this exemption to our normal procedure we aim to ensure that **the ".github" directory on the `stable` is never "ahead" of that in the `develop` branch**.
Therefore contributors who wish to update files in ".github" should submit their changes to `develop` and then request their changes to be applied to the `stable` branch.

### Hotfixes

There may be circumstances in which a change to gem5 is deemed critical and
cannot wait for an official release (e.g., a high-priority bug fix). In these
circumstances a hotfix shall be made.

First, if a developer suspects a hotfix may be necessary then the issue
should be discussed on the gem5-dev mailing list. The community will decide
whether the issue is worthy of a hotfix, and the final decision should be
made by members of the PMC if there is no consensus. Assuming the hotfix is
permitted, the following steps will be taken:

1. A new branch with the prefix "hotfix-" will be created from the stable
branch. Only gem5 maintainers can create branches. If a non-maintainer requires
the creation of a hotfix branch then they should contact a gem5 maintainer.
2. The change shall be submitted to the hotfix branch via github. Full review,
as with any other change, will be required.
3. Once fully submitted, the hotfix branch shall be merged into both the
develop and the stable branch by a gem5 maintainer.
4. The stable branch will be tagged with the new version number; the same as
the last but with an incremented hotfix number (e.g., "v20.2.0.0" would
transition to "v20.2.0.1").
4. The hotfix branch will then be deleted.
5. The gem5-dev and the gem5-user mailing lists shall be notified of this
hotfix.
