# Release notes for the gem5art package

## v1.4.0

- Update version now that it's in gem5

## v1.3.1

- Minor fixes
- Update documentation
- Prepare for merging with main gem5 repository

## v1.3.0

### Database now configurable

- Instead of only working with MongoDB installed at localhost, you can now specify the database connection parameter.
- You can specify it by explicitly calling `artifact.getDBConnection()` or using the `GEM5ART_DB` environment variable.
- The default is still `mongodb://localhost:271017`.
- All functions that query the database now *require* a `db` parameter (e.g., `getRuns()`).
- Reorganized some of the db functions in artifact, but this shouldn't affect end users.

### Other changes

- General documentation updates
