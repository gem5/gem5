
#ifndef __CPU_O3_DEP_GRAPH_HH__
#define __CPU_O3_DEP_GRAPH_HH__

#include "cpu/o3/comm.hh"

template <class DynInstPtr>
class DependencyEntry
{
  public:
    DependencyEntry()
        : inst(NULL), next(NULL)
    { }

    DynInstPtr inst;
    //Might want to include data about what arch. register the
    //dependence is waiting on.
    DependencyEntry<DynInstPtr> *next;
};

template <class DynInstPtr>
class DependencyGraph
{
  public:
    typedef DependencyEntry<DynInstPtr> DepEntry;

    DependencyGraph()
        : numEntries(0), memAllocCounter(0), nodesTraversed(0), nodesRemoved(0)
    { }

    void resize(int num_entries);

    void reset();

    void insert(PhysRegIndex idx, DynInstPtr &new_inst);

    void setInst(PhysRegIndex idx, DynInstPtr &new_inst)
    { dependGraph[idx].inst = new_inst; }

    void clearInst(PhysRegIndex idx)
    { dependGraph[idx].inst = NULL; }

    void remove(PhysRegIndex idx, DynInstPtr &inst_to_remove);

    DynInstPtr pop(PhysRegIndex idx);

    bool empty(PhysRegIndex idx) { return !dependGraph[idx].next; }

    /** Debugging function to dump out the dependency graph.
     */
    void dump();

  private:
    /** Array of linked lists.  Each linked list is a list of all the
     *  instructions that depend upon a given register.  The actual
     *  register's index is used to index into the graph; ie all
     *  instructions in flight that are dependent upon r34 will be
     *  in the linked list of dependGraph[34].
     */
    DepEntry *dependGraph;

    int numEntries;

    // Debug variable, remove when done testing.
    unsigned memAllocCounter;

  public:
    uint64_t nodesTraversed;
    uint64_t nodesRemoved;
};

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::resize(int num_entries)
{
    numEntries = num_entries;
    dependGraph = new DepEntry[numEntries];
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::reset()
{
    // Clear the dependency graph
    DepEntry *curr;
    DepEntry *prev;

    for (int i = 0; i < numEntries; ++i) {
        curr = dependGraph[i].next;

        while (curr) {
            memAllocCounter--;

            prev = curr;
            curr = prev->next;
            prev->inst = NULL;

            delete prev;
        }

        if (dependGraph[i].inst) {
            dependGraph[i].inst = NULL;
        }

        dependGraph[i].next = NULL;
    }
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::insert(PhysRegIndex idx, DynInstPtr &new_inst)
{
    //Add this new, dependent instruction at the head of the dependency
    //chain.

    // First create the entry that will be added to the head of the
    // dependency chain.
    DepEntry *new_entry = new DepEntry;
    new_entry->next = dependGraph[idx].next;
    new_entry->inst = new_inst;

    // Then actually add it to the chain.
    dependGraph[idx].next = new_entry;

    ++memAllocCounter;
}


template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::remove(PhysRegIndex idx,
                                    DynInstPtr &inst_to_remove)
{
    DepEntry *prev = &dependGraph[idx];
    DepEntry *curr = dependGraph[idx].next;

    // Make sure curr isn't NULL.  Because this instruction is being
    // removed from a dependency list, it must have been placed there at
    // an earlier time.  The dependency chain should not be empty,
    // unless the instruction dependent upon it is already ready.
    if (curr == NULL) {
        return;
    }

    nodesRemoved++;

    // Find the instruction to remove within the dependency linked list.
    while (curr->inst != inst_to_remove) {
        prev = curr;
        curr = curr->next;
        nodesTraversed++;

        assert(curr != NULL);
    }

    // Now remove this instruction from the list.
    prev->next = curr->next;

    --memAllocCounter;

    // Could push this off to the destructor of DependencyEntry
    curr->inst = NULL;

    delete curr;
}

template <class DynInstPtr>
DynInstPtr
DependencyGraph<DynInstPtr>::pop(PhysRegIndex idx)
{
    DepEntry *node;
    node = dependGraph[idx].next;
    DynInstPtr inst = NULL;
    if (node) {
        inst = node->inst;
        dependGraph[idx].next = node->next;
        node->inst = NULL;
        memAllocCounter--;
        delete node;
    }
    return inst;
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::dump()
{
    DepEntry *curr;

    for (int i = 0; i < numEntries; ++i)
    {
        curr = &dependGraph[i];

        if (curr->inst) {
            cprintf("dependGraph[%i]: producer: %#x [sn:%lli] consumer: ",
                    i, curr->inst->readPC(), curr->inst->seqNum);
        } else {
            cprintf("dependGraph[%i]: No producer. consumer: ", i);
        }

        while (curr->next != NULL) {
            curr = curr->next;

            cprintf("%#x [sn:%lli] ",
                    curr->inst->readPC(), curr->inst->seqNum);
        }

        cprintf("\n");
    }
    cprintf("memAllocCounter: %i\n", memAllocCounter);
}

#endif // __CPU_O3_DEP_GRAPH_HH__
