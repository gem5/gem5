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
def toFloat(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('Ei'):
        return float(value[:-2]) * exbi
    elif value.endswith('Pi'):
        return float(value[:-2]) * pebi
    elif value.endswith('Ti'):
        return float(value[:-2]) * tebi
    elif value.endswith('Gi'):
        return float(value[:-2]) * gibi
    elif value.endswith('Mi'):
        return float(value[:-2]) * mebi
    elif value.endswith('ki'):
        return float(value[:-2]) * kibi
    elif value.endswith('E'):
        return float(value[:-1]) * exa
    elif value.endswith('P'):
        return float(value[:-1]) * peta
    elif value.endswith('T'):
        return float(value[:-1]) * tera
    elif value.endswith('G'):
        return float(value[:-1]) * giga
    elif value.endswith('M'):
        return float(value[:-1]) * mega
    elif value.endswith('k'):
        return float(value[:-1]) * kilo
    elif value.endswith('m'):
        return float(value[:-1]) * milli
    elif value.endswith('u'):
        return float(value[:-1]) * micro
    elif value.endswith('n'):
        return float(value[:-1]) * nano
    elif value.endswith('p'):
        return float(value[:-1]) * pico
    elif value.endswith('f'):
        return float(value[:-1]) * femto
    else:
        return float(value)

def toLong(value):
    value = toFloat(value)
    result = int(value)
    if value != result:
        raise ValueError, "cannot convert '%s' to long" % value

    return result

def toInteger(value):
    value = toFloat(value)
    result = int(value)
    if value != result:
        raise ValueError, "cannot convert '%s' to integer" % value

    return result

def toBool(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    value = value.lower()
    if value == "true" or value == "t" or value == "yes" or value == "y":
        return True
    elif value == "false" or value == "f" or value == "no" or value == "n":
        return False

    raise ValueError, "cannot convert '%s' to bool" % value

def toFrequency(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('THz'):
        return float(value[:-3]) * tera
    elif value.endswith('GHz'):
        return float(value[:-3]) * giga
    elif value.endswith('MHz'):
        return float(value[:-3]) * mega
    elif value.endswith('kHz'):
        return float(value[:-3]) * kilo
    elif value.endswith('Hz'):
        return float(value[:-2])

    raise ValueError, "cannot convert '%s' to frequency" % value

def toLatency(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('ps'):
        return float(value[:-2]) * pico
    elif value.endswith('ns'):
        return float(value[:-2]) * nano
    elif value.endswith('us'):
        return float(value[:-2]) * micro
    elif value.endswith('ms'):
        return float(value[:-2]) * milli
    elif value.endswith('s'):
        return float(value[:-1])

    raise ValueError, "cannot convert '%s' to latency" % value

def toClockPeriod(value):
    """result is a clock period"""

    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    try:
        val = toFrequency(value)
        if val != 0:
            val = 1 / val
        return val
    except ValueError:
        pass

    try:
        val = toLatency(value)
        return val
    except ValueError:
        pass

    raise ValueError, "cannot convert '%s' to clock period" % value


def toNetworkBandwidth(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('Tbps'):
        return float(value[:-3]) * tera
    elif value.endswith('Gbps'):
        return float(value[:-3]) * giga
    elif value.endswith('Mbps'):
        return float(value[:-3]) * mega
    elif value.endswith('kbps'):
        return float(value[:-3]) * kilo
    elif value.endswith('bps'):
        return float(value[:-2])
    else:
        return float(value)

    raise ValueError, "cannot convert '%s' to network bandwidth" % value

def toMemoryBandwidth(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('PB/s'):
        return float(value[:-4]) * pebi
    elif value.endswith('TB/s'):
        return float(value[:-4]) * tebi
    elif value.endswith('GB/s'):
        return float(value[:-4]) * gibi
    elif value.endswith('MB/s'):
        return float(value[:-4]) * mebi
    elif value.endswith('kB/s'):
        return float(value[:-4]) * kibi
    elif value.endswith('B/s'):
        return float(value[:-3])

    raise ValueError, "cannot convert '%s' to memory bandwidth" % value

def toMemorySize(value):
    if not isinstance(value, str):
        raise TypeError, "wrong type '%s' should be str" % type(value)

    if value.endswith('PB'):
        return float(value[:-2]) * pebi
    elif value.endswith('TB'):
        return float(value[:-2]) * tebi
    elif value.endswith('GB'):
        return float(value[:-2]) * gibi
    elif value.endswith('MB'):
        return float(value[:-2]) * mebi
    elif value.endswith('kB'):
        return float(value[:-2]) * kibi
    elif value.endswith('B'):
        return float(value[:-1])

    raise ValueError, "cannot convert '%s' to memory size" % value
