#!/usr/bin/env python

import numpy as np

"""
Basically I have a single constant number
Add gaussian noise to it
Run linear least squares on it, and see what we get
"""

# get random number and print
# get a vector of 50 random samples of the number
# run least squares on it and see how close we get

rand_num = np.random.randint(0,100)
print rand_num

print np.random.normal(rand_num, 10, size=50)