# metric prefixes
exa  = 1.0e18
peta = 1.0e15
tera = 1.0e12
giga = 1.0e9
mega = 1.0e6
kilo = 1.0e3

milli = 1.0e-3
micro = 1.0e-6
nano  = 1.0e-9
pico  = 1.0e-12
femto = 1.0e-15
atto  = 1.0e-18

# power of 2 prefixes
kibi = 1024
mebi = kibi * 1024
gibi = mebi * 1024
tebi = gibi * 1024
pebi = tebi * 1024
exbi = pebi * 1024

# memory size configuration stuff
def to_integer(value):
    if not isinstance(value, str):
        result = int(value)
    elif value.endswith('Ei'):
        result = int(value[:-2]) * exbi
    elif value.endswith('Pi'):
        result = int(value[:-2]) * pebi
    elif value.endswith('Ti'):
        result = int(value[:-2]) * tebi
    elif value.endswith('Gi'):
        result = int(value[:-2]) * gibi
    elif value.endswith('Mi'):
        result = int(value[:-2]) * mebi
    elif value.endswith('ki'):
        result = int(value[:-2]) * kibi
    elif value.endswith('E'):
        result = int(value[:-1]) * exa
    elif value.endswith('P'):
        result = int(value[:-1]) * peta
    elif value.endswith('T'):
        result = int(value[:-1]) * tera
    elif value.endswith('G'):
        result = int(value[:-1]) * giga
    elif value.endswith('M'):
        result = int(value[:-1]) * mega
    elif value.endswith('k'):
        result = int(value[:-1]) * kilo
    elif value.endswith('m'):
        result = int(value[:-1]) * milli
    elif value.endswith('u'):
        result = int(value[:-1]) * micro
    elif value.endswith('n'):
        result = int(value[:-1]) * nano
    elif value.endswith('p'):
        result = int(value[:-1]) * pico
    elif value.endswith('f'):
        result = int(value[:-1]) * femto
    else:
        result = int(value)

    return result

def to_bool(val):
    t = type(val)
    if t == bool:
        return val

    if t == None:
        return False

    if t == int or t == long:
        return bool(val)

    if t == str:
        val = val.lower()
        if val == "true" or val == "t" or val == "yes" or val == "y":
            return True
        elif val == "false" or val == "f" or val == "no" or val == "n":
            return False

    return to_integer(val) != 0

def to_frequency(value):
    if not isinstance(value, str):
        result = float(value)
    elif value.endswith('THz'):
        result = float(value[:-3]) * tera
    elif value.endswith('GHz'):
        result = float(value[:-3]) * giga
    elif value.endswith('MHz'):
        result = float(value[:-3]) * mega
    elif value.endswith('kHz'):
        result = float(value[:-3]) * kilo
    elif value.endswith('Hz'):
        result = float(value[:-2])
    else:
        result = float(value)

    return result

def to_latency(value):
    if not isinstance(value, str):
        result = float(value)
    elif value.endswith('c'):
        result = float(value[:-1])
    elif value.endswith('ps'):
        result = float(value[:-2]) * pico
    elif value.endswith('ns'):
        result = float(value[:-2]) * nano
    elif value.endswith('us'):
        result = float(value[:-2]) * micro
    elif value.endswith('ms'):
        result = float(value[:-2]) * milli
    elif value.endswith('s'):
        result = float(value[:-1])
    else:
        result = float(value)

    return result;

def to_network_bandwidth(value):
    if not isinstance(value, str):
        result = float(value)
    elif value.endswith('Tbps'):
        result = float(value[:-3]) * tera
    elif value.endswith('Gbps'):
        result = float(value[:-3]) * giga
    elif value.endswith('Mbps'):
        result = float(value[:-3]) * mega
    elif value.endswith('kbps'):
        result = float(value[:-3]) * kilo
    elif value.endswith('bps'):
        result = float(value[:-2])
    else:
        result = float(value)

    return result

def to_memory_bandwidth(value):
    if not isinstance(value, str):
        result = int(value)
    elif value.endswith('PB/s'):
        result = int(value[:-4]) * pebi
    elif value.endswith('TB/s'):
        result = int(value[:-4]) * tebi
    elif value.endswith('GB/s'):
        result = int(value[:-4]) * gibi
    elif value.endswith('MB/s'):
        result = int(value[:-4]) * mebi
    elif value.endswith('kB/s'):
        result = int(value[:-4]) * kibi
    elif value.endswith('B/s'):
        result = int(value[:-3])
    else:
        result = int(value)

    return result

def to_memory_size(value):
    if not isinstance(value, str):
        result = int(value)
    elif value.endswith('PB'):
        result = int(value[:-2]) * pebi
    elif value.endswith('TB'):
        result = int(value[:-2]) * tebi
    elif value.endswith('GB'):
        result = int(value[:-2]) * gibi
    elif value.endswith('MB'):
        result = int(value[:-2]) * mebi
    elif value.endswith('kB'):
        result = int(value[:-2]) * kibi
    elif value.endswith('B'):
        result = int(value[:-1])
    else:
        result = int(value)

    return result
