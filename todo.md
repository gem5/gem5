- [X] get the environment to build

- [X] add a dummy cache implementation
    could be a duplicate cache implementation from another one

- [ ] build with that

- [ ] insure it's actually callable
    exposed to python config file

- [ ] figure out the difference between the cache implementation and the cache replacement policy
    there may be a need for us to implement a new type of cache (need 2 caches for this lfru policy)
    otherwise, just have some way to convey the fact that there are 2 cache types under the hood.
    hopefully, we can just reuse some of the logic from the other implementations.

- [ ] implement the lfru implementation
- [ ] cleanup git history
- [ ] merge it into the develop branch

Should move over lfru_rp into the replacement policies folder
