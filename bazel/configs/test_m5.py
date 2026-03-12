import sys

import _m5

print("Loaded _m5 successfully!", file=sys.stderr)
try:
    print(dir(_m5))
    print(dir(_m5.enum_MemoryMode))
except Exception as e:
    print("Error:", e, file=sys.stderr)
