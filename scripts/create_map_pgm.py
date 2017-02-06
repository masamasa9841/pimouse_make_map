#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

def map_line(length = 0):
    global f
    for i in range(length):
        f.write('100\n')
    
def map_line_2(x_length = 0, c = 0):
    global f
    for i in range(1,x_length+1):
        if i % 19 == 0 or i == 1:
            if c  % 19 == 0 or i == x_length or i == 1:
                f.write('100\n')
            else: f.write('0\n')
        else: f.write('0\n')

if __name__ == '__main__':
    print "write map scale"
    x = raw_input('x: ')
    y = raw_input('y: ')
    x_length = int(x) * 19
    y_length = int(y) * 19
    f = open('../map/map_origin.pgm','w')
    f.write('P2\n')
    f.write('#'+str(x)+','+str(y)+'\n')
    f.write(str(x_length)+' '+str(y_length)+'\n')
    f.write('255\n')
    map_line(x_length)
    for i in range(y_length-2):
        map_line_2(x_length, i+1)
    map_line(x_length)
    f.close()
