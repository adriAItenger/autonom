#!/usr/bin/python3

import math

def filterInf(ranges):
    filteredranges=[]
    for i in range(len(ranges)):
        if not math.isinf(ranges[i]):
            filteredranges.append([i, ranges[i]])

    return filteredranges

def getFarthestNeighbours(filteredranges,angles):
    p1, p2 = 0, 0
    maxdiff = 0
    for i in range(len(filteredranges) - 1):
        if abs(filteredranges[i][1] - filteredranges[i+1][1]) > maxdiff and 2<angles[i]<5: #itt a kettot es az 5-ot at kell irni ugy,
            p1 = filteredranges[i][0]                                                       #hogy elore nezzen 90 fokot mondjuk.
            p2 = filteredranges[i+1][0]
            maxdiff = abs(filteredranges[i][1] - filteredranges[i+1][1])

    if abs(filteredranges[0][1] - filteredranges[-1][1]) > maxdiff:
        p1 = filteredranges[-1][0]
        p2 = filteredranges[0][0]
        maxdiff = abs(filteredranges[0][1] - filteredranges[-1][1])
   
    return p1, p2

a = [13, 18, 21, float('inf'), float('inf'), 9, 7]
b=[1,2,3,4,5,6,7]
print(a)
print(getFarthestNeighbours(filterInf(a),b))
c,d=getFarthestNeighbours(filterInf(a),b)
print(c)
print(d)
