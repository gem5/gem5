from mercurial import context
from mercurial.i18n import _

'''
[extensions]
hgfilesize=~/m5/incoming/util/hgfilesize.py

[hooks]
pretxncommit = python:hgfilesize.limit_file_size
pretxnchangegroup = python:hgfilesize.limit_file_size

[limit_file_size]
maximum_file_size = 200000
'''

def limit_file_size(ui, repo, node=None, **kwargs):
    '''forbid files over a given size'''

    # default limit is 1 MB
    limit = int(ui.config('limit_file_size', 'maximum_file_size', 1024*1024))
    existing_tip = context.changectx(repo, node).rev()
    new_tip = context.changectx(repo, 'tip').rev()
    for rev in xrange(existing_tip, new_tip + 1):
        ctx = context.changectx(repo, rev)
        for f in ctx.files():
            if f not in ctx:
                continue
            fctx = ctx.filectx(f)
            if fctx.size() > limit:
                ui.write(_('file %s of %s is too large: %d > %d\n') % \
                             (f, ctx, fctx.size(), limit))
                return True # This is invalid

    return False # Things are OK.
