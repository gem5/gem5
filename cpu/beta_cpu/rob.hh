// Todo: Probably add in support for scheduling events (more than one as
// well) on the case of the ROB being empty or full.  Considering tracking
// free entries instead of insts in ROB.  Differentiate between squashing
// all instructions after the instruction, and all instructions after *and*
// including that instruction.

#ifndef __CPU_BETA_CPU_ROB_HH__
#define __CPU_BETA_CPU_ROB_HH__

#include <utility>
#include <vector>

//#include "arch/alpha/isa_traits.hh"

/**
 * ROB class.  Uses the instruction list that exists within the CPU to
 * represent the ROB.  This class doesn't contain that list, but instead
 * a pointer to the CPU to get access to the list.  The ROB, in this first
 * implementation, is largely what drives squashing.
 */
template <class Impl>
class ROB
{
  public:
    //Typedefs from the Impl.
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;

    typedef std::pair<RegIndex, PhysRegIndex> UnmapInfo_t;
    typedef typename list<DynInstPtr>::iterator InstIt_t;

  public:
    /** ROB constructor.
     *  @params _numEntries Number of entries in ROB.
     *  @params _squashWidth Number of instructions that can be squashed in a
     *                       single cycle.
     */
    ROB(unsigned _numEntries, unsigned _squashWidth);

    /** Function to set the CPU pointer, necessary due to which object the ROB
     *  is created within.
     *  @params cpu_ptr Pointer to the implementation specific full CPU object.
     */
    void setCPU(FullCPU *cpu_ptr);

    /** Function to insert an instruction into the ROB.  The parameter inst is
     *  not truly required, but is useful for checking correctness.  Note
     *  that whatever calls this function must ensure that there is enough
     *  space within the ROB for the new instruction.
     *  @params inst The instruction being inserted into the ROB.
     *  @todo Remove the parameter once correctness is ensured.
     */
    void insertInst(DynInstPtr &inst);

    /** Returns pointer to the head instruction within the ROB.  There is
     *  no guarantee as to the return value if the ROB is empty.
     *  @retval Pointer to the DynInst that is at the head of the ROB.
     */
    DynInstPtr readHeadInst() { return cpu->instList.front(); }

    DynInstPtr readTailInst() { return (*tail); }

    void retireHead();

    bool isHeadReady();

    unsigned numFreeEntries();

    bool isFull()
    { return numInstsInROB == numEntries; }

    bool isEmpty()
    { return numInstsInROB == 0; }

    void doSquash();

    void squash(InstSeqNum squash_num);

    uint64_t readHeadPC();

    uint64_t readHeadNextPC();

    InstSeqNum readHeadSeqNum();

    uint64_t readTailPC();

    InstSeqNum readTailSeqNum();

    /** Checks if the ROB is still in the process of squashing instructions.
     *  @retval Whether or not the ROB is done squashing.
     */
    bool isDoneSquashing() const { return doneSquashing; }

    /** This is more of a debugging function than anything.  Use
     *  numInstsInROB to get the instructions in the ROB unless you are
     *  double checking that variable.
     */
    int countInsts();

  private:

    /** Pointer to the CPU. */
    FullCPU *cpu;

    /** Number of instructions in the ROB. */
    unsigned numEntries;

    /** Number of instructions that can be squashed in a single cycle. */
    unsigned squashWidth;

    /** Iterator pointing to the instruction which is the last instruction
     *  in the ROB.  This may at times be invalid (ie when the ROB is empty),
     *  however it should never be incorrect.
     */
    InstIt_t tail;

    /** Iterator used for walking through the list of instructions when
     *  squashing.  Used so that there is persistent state between cycles;
     *  when squashing, the instructions are marked as squashed but not
     *  immediately removed, meaning the tail iterator remains the same before
     *  and after a squash.
     *  This will always be set to cpu->instList.end() if it is invalid.
     */
    InstIt_t squashIt;

    /** Number of instructions in the ROB. */
    int numInstsInROB;

    /** The sequence number of the squashed instruction. */
    InstSeqNum squashedSeqNum;

    /** Is the ROB done squashing. */
    bool doneSquashing;
};

#endif //__CPU_BETA_CPU_ROB_HH__
