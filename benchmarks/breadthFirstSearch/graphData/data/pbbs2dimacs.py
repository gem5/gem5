# converts graph input into dimacs format (for Galois MST)

import re
import sys
import random
filename = sys.argv[1]
f = open(filename,'r')
a = f.read()
b = a.split('\n')

b.pop(0)
b.pop()

numVertices = sys.argv[2]
numEdges = len(b)
print 'p sp ' + numVertices + ' ' + str(numEdges)

for line in b:
    points = line.split(' ')
    u = points[0]
    v = points[1]
    weightList = points[2].split('e')

    mul = float(pow(10,int(weightList[1])))
    weight = float(weightList[0]) * mul
    print 'a ' + str(u) + ' ' + str(v) + ' ' + str(weight*100)
