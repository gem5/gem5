# Add support for multiple event queues
def upgrader(cpt):
    cpt.set("Globals", "numMainEventQueues", "1")


legacy_version = 12
