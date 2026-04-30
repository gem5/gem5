/**
 * lvp_probe.c — LVP Training & Prediction Probe
 *
 * Phase 3 verification program for the FLOP/LVP research.
 *
 * Goal: train the Load Value Predictor past its confidence threshold (250)
 * by repeatedly loading the same address with the same value. After iteration
 * 250, the predictor becomes confident and starts forwarding predictions at the
 * Rename stage, visible in the LVP debug trace as:
 *
 *   [tid:0] Lookup PC=0xXXXX → confident prediction 0x2a (counter=250/250)
 *
 * Compile:
 *   aarch64-linux-gnu-gcc -O0 -static -o lvp_probe flop_research/phase3/lvp_probe.c
 */

#include <stdio.h>
#include <stdint.h>

/* volatile prevents the compiler from caching in a register across iterations.
 * 4-byte int — LVP filters out 8-byte pointer loads, so we use int32_t. */
volatile int32_t sensor_value = 42;

int main(void)
{
    int32_t sum = 0;
    int i;

    printf("LVP Probe: training predictor for 300 iterations...\n");

    /*
     * ── PHASE A: Training ────────────────────────────────────────────────
     * Run 260 iterations of the load.  Every iteration calls update() in
     * IEW, incrementing the counter for this load's PC.
     * After 260 iterations: counter = 255 (saturated at uint8_t max).
     *
     * The printf() syscall below acts as a COMMIT BARRIER:
     * gem5 must commit all prior loads before the syscall executes, so by
     * the time Phase B starts, the LVP counter is fully trained (=255).
     */
    for (i = 0; i < 300; i++) {
        sum += sensor_value;
    }
    //printf("Training complete: sum=%d, counter should be ~255\n", sum);

    /*
     * ── PHASE B: Prediction ──────────────────────────────────────────────
     * Now counter >= 250 for this load PC.  Each Rename lookup should
     * return a confident prediction (0x2a = 42).
     * The trace should show:
     *   [tid:0] Lookup PC=0xXXXX → confident prediction 0x2a (counter=255/250)
     * and update() confirms with predictionHits++, no squash.
     */
    //printf("Starting prediction phase...\n");
    // for (i = 0; i < 20; i++) {
    //     sum += sensor_value;
    // }
    //printf("Prediction phase complete: sum=%d\n", sum);

    /*
     * ── PHASE C: Deliberate Misprediction ───────────────────────────────
     * Tamper with the value.  LVP is still confident with value=42.
     * The next load returns 99 → misprediction → squashDueToMemOrder().
     * The trace should show:
     *   [tid:0] PC=0xXXXX: MISPREDICTION (predicted=0x2a, actual=0x63)
     */
    sensor_value = 99;
    sum += sensor_value;   /* <-- LVP predicts 42, real is 99 → SQUASH */
    printf("After tamper (expect SQUASH in trace): sum=%d\n", sum);

    printf("Done.\n");
    return 0;
}
