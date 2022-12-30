## Gerrit Bot

### Getting Username and Password
Gerrit REST API uses the account username and password for the authentication
purpose. They are necessary to make a request.

The following are steps to obtain the username and password from `.gitcookies`
files,

* Follow this link
[https://gem5-review.googlesource.com/new-password](https://gem5-review.googlesource.com/new-password)
and copy the authenticating script to a new file.

* After that, run the `extract_gitcookies.py` to extract the username and
password from the authenticating script.
For example, the following command extracts the username and password from
`gerrit_auth_script` and writes them to `.data/auth`,
```sh
python3 extract_gitcookies.py gerrit_auth_script .data/auth
```
The `.data/auth` will have two lines: the first line contains the username and
the second line is the corresponding password.
**Notes:**
* The above link, [https://gem5-review.googlesource.com/new-password](https://gem5-review.googlesource.com/new-password),
generates a new pair of username and password per visit.
* The `extract_gitcookies.py` file is also able to read from `.gitcookies`
file. For example, `python3 extract_gitcookies.py ~/.gitcookies output`
will write all pairs of username and password in two lines per pair to
`output`.
* The gerrit-bot only reads the pair of username and password appearing
in the first and the second line in the `.data/auth` file.

### Gerrit Bot

The structure of the Gerrit bot is as follows:
* The `GerritBotConfig` class should contain all constants that are
configurable prior to running.

### Gerrit API
* Query options: [https://gerrit-review.googlesource.com/Documentation/rest-api-changes.html#query-options](https://gerrit-review.googlesource.com/Documentation/rest-api-changes.html#query-options)

### Deployment
The Gerrit bot is intended to be run as a cron job.
Each run of the Gerrit bot will query new changes made to the Gerrit server
within a certain period of time, perform actions on each change, and exit.

The following are steps to deploy the Gerrit bot:

* Create `.data` folder in the same folder as `bot.py`,
```sh
mkdir .data
```

* Follow the steps [here](#getting-username-and-password) to get the Gerrit
bot account username and password.

* To run the Gerrit bot once,
```sh
./bot.py
```

* To edit the cron table,
```sh
crontab -e
```

To run the Gerrit bot every 30 minutes, add the following line to the
crontable,
```python
*/1 * * * * cd /path/to/gerrit/bot/directory && ./bot.py
```
