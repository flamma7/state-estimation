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
num_meas = 1000

rand_const = np.random.randint(0,100)
print rand_const

measurements = np.random.normal(rand_const, 10, size=(num_meas,1))
ones = np.ones((num_meas, 1))
hl = np.linalg.pinv(ones)
estimate = np.dot(hl, measurements)
print estimate