
#ifndef __CPU_ACTIVITY_HH__
#define __CPU_ACTIVITY_HH__

#include "base/timebuf.hh"
#include "base/trace.hh"

class ActivityRecorder {
  public:
    ActivityRecorder(int num_stages, int longest_latency, int count);

    /** Records that there is activity this cycle. */
    void activity();
    /** Advances the activity buffer, decrementing the activityCount if active
     *  communication just left the time buffer, and descheduling the CPU if
     *  there is no activity.
     */
    void advance();
    /** Marks a stage as active. */
    void activateStage(const int idx);
    /** Deactivates a stage. */
    void deactivateStage(const int idx);

    int getActivityCount() { return activityCount; }

    void setActivityCount(int count)
    { activityCount = count; }

    bool active() { return activityCount; }

    void reset();

    void dump();

    void validate();

  private:
    /** Time buffer that tracks if any cycles has active communication
     *  in them.  It should be as long as the longest communication
     *  latency in the system.  Each time any time buffer is written,
     *  the activity buffer should also be written to. The
     *  activityBuffer is advanced along with all the other time
     *  buffers, so it should have a 1 somewhere in it only if there
     *  is active communication in a time buffer.
     */
    TimeBuffer<bool> activityBuffer;

    int longestLatency;

    /** Tracks how many stages and cycles of time buffer have
     *  activity. Stages increment this count when they switch to
     *  active, and decrement it when they switch to
     *  inactive. Whenever a cycle that previously had no information
     *  is written in the time buffer, this is incremented. When a
     *  cycle that had information exits the time buffer due to age,
     *  this count is decremented. When the count is 0, there is no
     *  activity in the CPU, and it can be descheduled.
     */
    int activityCount;

    int numStages;

    /** Records which stages are active/inactive. */
    bool *stageActive;
};

#endif // __CPU_ACTIVITY_HH__
