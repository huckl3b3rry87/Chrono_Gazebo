#!/bin/python

import math

R = 50
COUNT = 50
FILE_NAME = 'testpath.txt'

step = math.pi * 2 / 50
f = open(FILE_NAME, 'w')

f.write('%d 3\n' % COUNT)

for i in xrange(COUNT + 1):
	ang = step * i
	x = R * math.cos(ang)
	y = R * math.sin(ang)
	f.write('%0.6f\t%0.6f\t0.1\n' % (x, y))
