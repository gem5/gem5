import m5
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')
addToPath('../example')

from Caches import *

from optparse import OptionParser

from yattag import Doc

from fs import *

for i in range(np):
    print test_sys.cpu[i].workload

(doc, tag, text) = Doc().tagtext()

with tag('document'):
    with tag('component', id = 'root', name = 'root'):
        with tag('component', id = 'system', name = 'system', type = 'System'):
            with tag('param', name = 'core_tech_node', value = '40'):
                pass
            for i in range(np):
                with tag('component', id = 'system.core' + str(i), name = 'core' + str(i), type = 'Core'):
                    pass

print(doc.getvalue())



