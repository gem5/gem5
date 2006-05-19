
#include "base/timebuf.hh"
#include "cpu/activity.hh"

ActivityRecorder::ActivityRecorder(int num_stages, int longest_latency,
                                   int activity)
    : activityBuffer(longest_latency, 0), longestLatency(longest_latency),
      activityCount(activity), numStages(num_stages)
{
    stageActive = new bool[numStages];
    memset(stageActive, 0, numStages);
}

void
ActivityRecorder::activity()
{
    if (activityBuffer[0]) {
        return;
    }

    activityBuffer[0] = true;

    ++activityCount;

    DPRINTF(Activity, "Activity: %i\n", activityCount);
}

void
ActivityRecorder::advance()
{
    if (activityBuffer[-longestLatency]) {
        --activityCount;

        assert(activityCount >= 0);

        DPRINTF(Activity, "Activity: %i\n", activityCount);

        if (activityCount == 0) {
            DPRINTF(Activity, "No activity left!\n");
        }
    }

    activityBuffer.advance();
}

void
ActivityRecorder::activateStage(const int idx)
{
    if (!stageActive[idx]) {
        ++activityCount;

        stageActive[idx] = true;

        DPRINTF(Activity, "Activity: %i\n", activityCount);
    } else {
        DPRINTF(Activity, "Stage %i already active.\n", idx);
    }

//    assert(activityCount < longestLatency + numStages + 1);
}

void
ActivityRecorder::deactivateStage(const int idx)
{
    if (stageActive[idx]) {
        --activityCount;

        stageActive[idx] = false;

        DPRINTF(Activity, "Activity: %i\n", activityCount);
    } else {
        DPRINTF(Activity, "Stage %i already inactive.\n", idx);
    }

    assert(activityCount >= 0);
}

void
ActivityRecorder::reset()
{
    activityCount = 0;
    memset(stageActive, 0, numStages);
    for (int i = 0; i < longestLatency + 1; ++i)
        activityBuffer.advance();
}

void
ActivityRecorder::dump()
{
    for (int i = 0; i <= longestLatency; ++i) {
        cprintf("[Idx:%i %i] ", i, activityBuffer[-i]);
    }

    cprintf("\n");

    for (int i = 0; i < numStages; ++i) {
        cprintf("[Stage:%i %i]\n", i, stageActive[i]);
    }

    cprintf("\n");

    cprintf("Activity count: %i\n", activityCount);
}

void
ActivityRecorder::validate()
{
    int count = 0;
    for (int i = 0; i <= longestLatency; ++i) {
        if (activityBuffer[-i]) {
            count++;
        }
    }

    for (int i = 0; i < numStages; ++i) {
        if (stageActive[i]) {
            count++;
        }
    }

    assert(count == activityCount);
}
