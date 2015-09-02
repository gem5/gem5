# Add block_size_bytes to system.ruby
def upgrader(cpt):
    for sec in cpt.sections():
        if sec == 'system.ruby':
            # Use Gem5's default of 64; this should be changed if the to be
            # upgraded checkpoints were not taken with block-size 64!
            cpt.set(sec, 'block_size_bytes', '64')

legacy_version = 10
