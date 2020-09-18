## Gerrit Bot

### Getting Username and Password

* Follow this link
[https://gem5-review.googlesource.com/new-password](https://gem5-review.googlesource.com/new-password)
and copy the authenticating script to a file, supposedly `gerrit_auth_script`.

* After that, run the `extract_gitcookies.py` to extract the username and
password from the obtained script.
For example, the following command extracts the username and password from
`gerrit_auth_script` and writes them to `GEM5_BOT_AUTH_INFO`,
```sh
python3 extract_gitcookies.py gerrit_auth_script GEM5_BOT_AUTH_INFO
```
The `GEM5_BOT_AUTH_INFO` will have two lines: the first line contains the
username and the second line is the corresponding password.
**Notes:**
* The above link, [https://gem5-review.googlesource.com/new-password](https://gem5-review.googlesource.com/new-password),
generates a new pair of username and password per visit.
* The `extract_gitcookies.py` file is also able to read from `.gitcookies`
file. For example, `python3 extract_gitcookies.py ~/.gitcookies output`
will write all pairs of username and password in two lines per pair to
`output`.
* The gerrit-bot only reads the pair of username and password appearing
in the first and the second line in the `GEM5_BOT_AUTH_INFO` file.

### Gerrit Bot

**Notes:** this is likely to be changed.

The idea of implementing the bot is as follows,
* The `Configs` class should contain all constants that are configurable
prior to running.
* Classes such as `LabelInfo` and `ReviewInput` are simplied versions
resembling those data structures of the same name according to the
[Gerrit REST API documentation](https://gerrit-review.googlesource.com/Documentation/rest-api.html#_endpoints).
* In the class `GerritRestAPIRequester`,
    * The `__generate_*_request()` functions should be a one-to-one function
to a set of Gerrit REST API endpoints. The functions should generate a
`requests.Request` object.
    * The `send_*()` functions are adapted to a more specific use case.

### Gerrit API
* Query options: [https://gerrit-review.googlesource.com/Documentation/rest-api-changes.html#query-options](https://gerrit-review.googlesource.com/Documentation/rest-api-changes.html#query-options)

### Appendix I. `extract_gitcookies.py`
This script extracts all pairs of username and password from the gerrit
authentication script from a file or a .gitcookies file.

The usage of the script is as follows,
```sh
python3 extract_gitcookies.py input_path output_path
```

### Appendix II. `MAINTAINERS.json`
This file should be consistent with the file `MAINTAINERS`.
