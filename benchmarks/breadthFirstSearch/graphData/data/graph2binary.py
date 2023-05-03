import sys
import random
from struct import *

filename = sys.argv[1]
out = sys.argv[2]
f = open(filename, "r")
a = f.read()
b = a.split("\n")

b.pop(0)

numVertices = int(b[0])
numEdges = int(b[1])

# print numVertices, numEdges
"""
for x in range(numVertices):
    offset = int(b[2+x])
    if x == numVertices-1:
        length = numEdges - offset
    else:
        length = int(b[2+x+1]) - offset
    adj = [0]*numEdges
    #line = '';
    for i in range(length):
        adj[int(b[2+numVertices+offset+i])] = 1
        #line += str(b[2+numVertices+offset+i]) + ' '
    #print line
    line = '';
    for i in range(numEdges):
        line += str(adj[i]) + ' '
    print line
"""

rowArray = []
colArray = []
for x in range(numVertices):
    offset = int(b[2 + x])
    if x == numVertices - 1:
        length = numEdges - offset
    else:
        length = int(b[2 + x + 1]) - offset
    for i in range(length):
        rowArray.append(x)
        colArray.append(int(b[2 + numVertices + offset + i]))


v = [1] * numEdges

# print numVertices,numEdges
# print len(colArray)
# print len(rowArray)
# print v

ff = open(out, "wb")
ff.write(pack("III", numVertices, numVertices, numEdges))
ff.write(pack(numEdges * "I", *rowArray))
ff.write(pack(numEdges * "I", *colArray))
ff.write(pack(numEdges * "d", *v))

# print rowArray
# print colArray
# print v
