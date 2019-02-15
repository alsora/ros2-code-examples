#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


msg_size = [
    0.01,
    1,
    5,
    10,
    50,
    100,
    250,
    300,
    400,
    450,
    500,
    750,
    1000,
    2000,
    4000]


physical_mem = [
    20.776,
    21.008,
    21.836,
    22.112,
    26.032,
    30.220,
    42.700,
    46.076,
    53.100,
    56.444,
    21.392,
    21.796,
    21.484,
    21.616,
    21.460]

virtual_mem =[
    420.356,
    420.620,
    421.364,
    422.344,
    429.588,
    438.800,
    466.584,
    475.828,
    494.296,
    503.532,
    513.020,
    537.692,
    605.276,
    789.788,
    1158.816]


f, axarr = plt.subplots(2)
f.subplots_adjust(hspace=.75)

axarr[0].plot(msg_size, physical_mem, marker='o')
axarr[0].set_xscale('log')
axarr[0].set_title('Physical memory')
axarr[0].set_xlabel('Message size [KB]')
axarr[0].set_ylabel('Memory [MB]')

axarr[1].plot(msg_size, virtual_mem, marker='o')
axarr[1].set_xscale('log')
axarr[1].set_title('Virtual memory')
axarr[1].set_xlabel('Message size [KB]')
axarr[1].set_ylabel('Memory [MB]')


plt.show()