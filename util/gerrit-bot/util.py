# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Utility functions
def parse_commit_subject(subject):
    parsed_subject = subject.split(":", maxsplit = 1)

    # If the subject does not have a colon, it either does not have tags
    # or does not have a message. In this case, we assume that the subject
    # is the commit message.
    if len(parsed_subject) <= 1:
        return [], parsed_subject[0]

    tags = [ tag.strip() for tag in parsed_subject[0].split(",") ]
    message = parsed_subject[1]

    return tags, message

# Convert time in seconds to a plausible unit
def convert_time_in_seconds(delta):
    time = int(delta)
    time_unit = "s"

    for curr_unit_limit, next_unit in zip([60, 60, 24], ["m", "h", "d"]):
        if time <= curr_unit_limit:
            break
        time = time // curr_unit_limit + 1
        time_unit = next_unit

    return f"{time}{time_unit}"

# End of Utility functions

def add_maintainers_to_change(change, maintainers, maintainers_account_ids,
                              gerrit_api):
    tags, message = parse_commit_subject(change["subject"])
    change_id = change["id"]
    maintainer_emails = set()

    # There are cases that a reviewer being removed from the reviewer list
    # by another reviewer. We want to respect this removal. To do this,
    # we can avoid adding reviewers that have been added/removed to the
    # reviewer list.
    avoid_emails = set()
    for update in change["reviewer_updates"]:
        avoid_emails.add(update["reviewer"]["email"])
    for tag in tags:
        try:
            for name, email in maintainers[tag].maintainers:
                maintainer_emails.add(email)
        except KeyError:
            print((f"warning: `change-{change_id}` has an unknown tag: "
                   f"`{tag}`"))
    for email in maintainer_emails:
        if email in avoid_emails:
            continue
        try:
            account_id = maintainers_account_ids[email]
            gerrit_api.add_reviewer(change_id, account_id)
        except KeyError:
            # if cannot find the account id of a maintainer
            # then use the email address
            gerrit_api.add_reviewer(change_id, email)
