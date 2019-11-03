def main():
    from _m5.stattest import stattest_init, stattest_run
    import m5.stats

    stattest_init()

    # Initialize the global statistics
    m5.stats.initSimStats()
    m5.stats.addStatVisitor("cout")

    # We're done registering statistics.  Enable the stats package now.
    m5.stats.enable()

    # Reset to put the stats in a consistent state.
    m5.stats.reset()

    stattest_run()

    m5.stats.dump()
